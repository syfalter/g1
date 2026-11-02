"""Prepare the existing 9mm candidate for activation from the verified 7mm release."""
import hashlib
import json
from pathlib import Path
import shutil


def main():
    root = Path('/home/unitree/rh56dfx/workspace/execution/action_runtime_v3')
    candidates = root.parent / 'trajectory_candidates'
    series = candidates / 'push-depth-trial-series-20260929'
    source = series / 'trial_9mm'
    stage = candidates / 'push9-activation-20260930'
    read = lambda path: json.loads(path.read_text())
    live = read(root / 'config/selected_bundle.json')
    expected = read(series / 'trial_7mm/data_only/config/selected_bundle.json')
    if live != expected or live['sha256'] != '9fbd5057316b63e72de41a098714e4259d0bebd6ce237f326b144bb81da3e16b':
        raise RuntimeError('Current release is not the verified 7mm version')
    new = read(source / 'data_only/config/selected_bundle.json')
    for name, action in live['payload']['actions'].items():
        if name != 'push' and action != new['payload']['actions'][name]:
            raise RuntimeError('Unrelated action changed: ' + name)
    for key in live['payload']:
        if key not in ('actions', 'version', 'parents') and live['payload'][key] != new['payload'][key]:
            raise RuntimeError('Unrelated release field changed: ' + key)
    receipt = read(source / 'jetson_readonly/receipt.json')
    for name, digest in receipt['hashes'].items():
        if name != 'config/selected_bundle.json' and hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Inspected library or runtime changed: ' + name)
    shutil.copytree(source, stage)
    for name in receipt['hashes']:
        shutil.copy2(root / name, stage / 'jetson_readonly' / name)
        receipt['hashes'][name] = hashlib.sha256((root / name).read_bytes()).hexdigest()
    receipt['activation_base_release_sha256'] = live['sha256']
    (stage / 'jetson_readonly/receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    manifest = read(stage / 'export_report.json')
    manifest.update(user_confirmed_disabled=True, activation_base_release_sha256=live['sha256'],
                    activation_authorization_date='2026-09-30')
    (stage / 'export_report.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(stage)


if __name__ == '__main__':
    main()
