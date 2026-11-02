const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');

(async () => {
  const browser = await chromium.launch({headless:true,
    ...(process.env.CHROMIUM_PATH?{executablePath:process.env.CHROMIUM_PATH}:{})});
  const page = await browser.newPage({viewport:{width:1440,height:1100}});
  const errors=[];
  page.on('pageerror', error=>errors.push(error.message));
  const output=process.env.SCREENSHOT_DIR || '/tmp/rail-preview';
  fs.mkdirSync(output,{recursive:true});
  await page.goto(process.env.PREVIEW_URL || 'http://127.0.0.1:8790');
  await page.waitForFunction(()=>document.querySelector('#connection').textContent==='离线服务已连接');
  await page.locator('#initialize').click();
  await page.waitForFunction(()=>document.querySelector('#phase').textContent==='就绪');
  await page.locator('#air').click();
  await page.waitForFunction(()=>document.querySelector('#fault').disabled===false);
  await page.locator('#fault').click();
  await page.waitForFunction(()=>document.querySelector('#phase').textContent==='等待恢复');
  assert(await page.locator('#reset').isDisabled());
  await page.locator('#confirm-stop').click();
  await page.locator('#contact-clear').check();
  await page.locator('#support-ready').check();
  await page.locator('#inspect').click();
  await page.locator('#reset').click();
  await page.waitForFunction(()=>document.querySelector('#phase').textContent==='待初始化');
  await page.locator('#handoff-support').check();
  await page.locator('#release').click();
  await page.locator('#wsl-claim').click();
  await page.waitForFunction(()=>document.querySelector('#phase').textContent==='WSL 调试中');
  assert(await page.locator('#initialize').isDisabled());
  assert(await page.locator('#air').isDisabled());
  assert(await page.locator('#acquire').isDisabled());
  await page.locator('#wsl-release').click();
  await page.locator('#acquire').click();
  await page.waitForFunction(()=>document.querySelector('#phase').textContent==='待初始化');
  await page.locator('#initialize').click();
  await page.waitForFunction(()=>document.querySelector('#phase').textContent==='就绪');
  await page.locator('#curve-operation').selectOption('pull');
  await page.waitForFunction(()=>!document.querySelector('#message').classList.contains('visible'));
  await page.screenshot({path:path.join(output,'desktop.png'),fullPage:true});
  for(const viewport of [{width:390,height:844},{width:768,height:1024}]) {
    await page.setViewportSize(viewport);
    await page.waitForTimeout(250);
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    assert(await page.locator('#plot').evaluate(c=>{
      const a=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
      let drawn=0;for(let i=3;i<a.length;i+=4)if(a[i])drawn++;return drawn>1000;
    }));
    await page.screenshot({path:path.join(output,`${viewport.width}.png`),fullPage:true});
  }
  assert.deepEqual(errors,[]);
  await browser.close();
  console.log('Browser recovery, handoff, canvas and responsive checks passed');
})().catch(error=>{console.error(error);process.exit(1);});
