"""Deploy this verified build with a backup of every overwritten file."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

parser=argparse.ArgumentParser()
parser.add_argument('--target',type=Path,required=True)
parser.add_argument('--source',type=Path,required=True,help='Python source used to generate fixtures')
parser.add_argument('--apply',action='store_true')
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
target=args.target.resolve(); source=args.source.resolve()
snapshot=json.loads((root/'validation/python_source_snapshot.json').read_text(encoding='utf-8'))
for name,expected in snapshot['files'].items():
    if digest(source/name)!=expected:
        raise RuntimeError(f'Python source changed during synchronization: {name}')
items={}
for folder in ('src','configs','models','tests','tools'):
    for file in (root/folder).rglob('*'):
        if file.is_file() and '__pycache__' not in file.parts:
            items[file.relative_to(root).as_posix()]=file
for name in ('README.md','CMakeLists.txt'):
    items[name]=root/name
for name in ('MediaPipe2ManoQt.exe','parity_tests.exe','sync_parity_tests.exe',
             'skeleton_parity_tests.exe','config_profiles_tests.exe'):
    items['bin/'+name]=root/'build/Release'/name
for file in (root/'validation').iterdir():
    if file.is_file() and file.name not in ('deploy_plan.json','deployment_receipt.json'):
        items['validation/sync-20260914/'+file.name]=file
changes={}
for name,file in items.items():
    destination=(target/name).resolve()
    if target not in destination.parents:
        raise RuntimeError('Destination escaped target project')
    sha=digest(file)
    if destination.is_file() and digest(destination)==sha:
        continue
    changes[name]=dict(sha256=sha,bytes=file.stat().st_size,
                       previous_sha256=digest(destination) if destination.is_file() else None)
plan=dict(target=str(target),python_commit=snapshot['commit'],changes=changes)
plan_path=root/'validation/deploy_plan.json'
if not args.apply:
    plan_path.write_text(json.dumps(plan,indent=2),encoding='utf-8')
    print(f'Reviewable update: {len(changes)} files, {sum(x["bytes"] for x in changes.values())} bytes')
else:
    expected=json.loads(plan_path.read_text(encoding='utf-8'))
    if plan!=expected:
        raise RuntimeError('Source/target changed after deployment plan; review a new plan')
    backup=target/'backups'/('before-python-sync-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.zip')
    backup.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(backup,'x',zipfile.ZIP_DEFLATED) as archive:
        for name in changes:
            previous=target/name
            if previous.is_file(): archive.write(previous,name)
        archive.writestr('update_manifest.json',json.dumps(plan,indent=2))
    for name in changes:
        destination=target/name
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(items[name],destination)
        if digest(destination)!=changes[name]['sha256']:
            raise RuntimeError(f'Deployed checksum mismatch: {name}')
    receipt=dict(backup=str(backup),files=len(changes),python_commit=snapshot['commit'])
    (target/'validation/sync-20260914/deployment_receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8')
    print(json.dumps(receipt))
