const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({headless:true,
    ...(process.env.CHROMIUM_PATH ? {executablePath:process.env.CHROMIUM_PATH} : {})});
  try {
    const page = await browser.newPage({viewport:{width:1440,height:1100}});
    const errors=[];
    page.on('pageerror', e=>errors.push(e.message));
    const base=process.env.PREVIEW_URL || 'http://127.0.0.1:8791';
    const get=async route=>(await page.request.get(base+route)).json();
    await page.goto(base);
    await page.waitForFunction(()=>document.querySelector('#connection').textContent==='离线服务已连接');
    let before=await get('/api/state');
    assert.equal(before.simulation,true);
    assert.equal(before.hardwareConnected,false);
    assert.equal(before.owner,'jetson');
    assert.equal(before.active,null);
    const version=process.env.PATCH_VERSION || '20260922-185858-push-161344';
    const session=await get('/api/session');
    const post=(route,body)=>page.request.post(base+route,{data:body,
      headers:{'X-Preview-Token':session.token,Origin:base}});
    const baseline=process.env.BASELINE_VERSION || 'baseline-20260921-v2';
    if(before.release_version!==baseline) {
      const restored=await post('/api/releases/activate',{version:baseline,expectedSha256:before.bundle});
      assert.equal(restored.status(),200);
      before=await get('/api/state');
    }
    await page.waitForFunction(old=>document.querySelector('#active-version').textContent===old,before.release_version);
    const initial=await get('/api/releases');
    const old=initial.releases.find(r=>r.version===before.release_version);
    assert(old,'Current full version must exist for rollback');
    await page.locator('#release-select').selectOption(version);
    const activation=page.waitForResponse(r=>r.url().endsWith('/api/releases/activate')&&r.request().method()==='POST');
    await page.locator('#activate-release').click();
    assert.equal((await activation).status(),200);
    await page.waitForFunction(old=>document.querySelector('#active-version').textContent!==old,before.release_version);
    const active=await get('/api/state');
    assert.equal(active.phase,'STARTUP_REQUIRED');
    const after=(await get('/api/releases')).releases.find(r=>r.version===active.release_version);
    for(const name of Object.keys(old.actions).filter(n=>n!=='push')) {
      assert.deepEqual(after.actions[name],old.actions[name],`${name} changed with a push-only patch`);
    }
    await page.locator('#initialize').click();
    await page.waitForFunction(()=>document.querySelector('#phase').textContent==='就绪');
    await page.locator('#action-select').selectOption('push');
    await page.locator('#run-action').click();
    await page.waitForFunction(()=>document.querySelector('#phase').textContent==='执行中');
    const denied=await post('/api/releases/activate',{version:before.release_version,expectedSha256:active.bundle});
    assert.equal(denied.status(),409);
    await page.waitForFunction(()=>document.querySelector('#phase').textContent==='就绪',null,{timeout:60000});
    await page.locator('#rollback-release').click();
    await page.waitForFunction(old=>document.querySelector('#active-version').textContent===old,before.release_version);
    assert.equal((await get('/api/state')).bundle,before.bundle);
    await page.locator('#initialize').click();
    await page.waitForFunction(()=>document.querySelector('#phase').textContent==='就绪');
    const output=process.env.SCREENSHOT_DIR || '/tmp/rail-release-preview';
    fs.mkdirSync(output,{recursive:true});
    for(const size of [{width:1440,height:1100},{width:390,height:844}]) {
      await page.setViewportSize(size);
      await page.waitForTimeout(300);
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
      await page.screenshot({path:path.join(output,`releases-${size.width}.png`),fullPage:true});
    }
    assert.deepEqual(errors,[]);
    console.log('PASS: patch application, other actions preserved, busy switch rejected, exact rollback, responsive UI');
  } finally {
    await browser.close();
  }
})().catch(e=>{console.error(e);process.exitCode=1;});
