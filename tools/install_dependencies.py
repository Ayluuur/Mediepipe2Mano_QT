"""Download official, pinned C++ build dependencies into this project only."""
import hashlib
import json
from pathlib import Path
import time
import urllib.request
import zipfile

root=Path(__file__).resolve().parents[1]
third_party=root/'ThirdParty'
third_party.mkdir(exist_ok=True)
assets=[
    ('open3d.zip','https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-windows-amd64-0.19.0.zip','open3d-devel-windows-amd64-0.19.0'),
    ('bazel.exe','https://releases.bazel.build/6.1.1/release/bazel-6.1.1-windows-x86_64.exe',None),
    ('mediapipe.zip','https://codeload.github.com/google-ai-edge/mediapipe/zip/refs/tags/v0.10.9','mediapipe-0.10.9'),
]
manifest={}
for name,url,folder in assets:
    path=third_party/name
    if not path.exists():
        for attempt in range(3):
            try:
                urllib.request.urlretrieve(url,path.with_suffix(path.suffix+'.part'))
                path.with_suffix(path.suffix+'.part').replace(path)
                break
            except Exception:
                if attempt==2:
                    raise
                time.sleep(2)
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    lock_path=root/'dependencies.lock.json'
    if lock_path.exists():
        expected=json.loads(lock_path.read_text(encoding='utf-8')).get(name,{}).get('sha256')
        if expected and expected!=digest:
            raise RuntimeError(f'Checksum mismatch: {path}')
    manifest[name]=dict(url=url,sha256=digest)
    if folder and not (third_party/folder).exists():
        with zipfile.ZipFile(path) as archive:
            for member in archive.infolist():
                destination=(third_party/member.filename).resolve()
                if third_party.resolve() not in destination.parents and destination!=third_party.resolve():
                    raise RuntimeError('Archive entry escapes dependency directory')
            archive.extractall(third_party)
    print(name,'ready',flush=True)
(root/'dependencies.installed.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
