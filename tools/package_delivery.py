"""Package the source, runtime, and pinned SDKs, excluding disposable build caches."""
import hashlib
import json
from pathlib import Path
import zipfile

root=Path(__file__).resolve().parents[1]
archive=root.parent/'Mediepipe2Mano_QT-delivery.zip'
folders=['src','tools','tests','models','assets','configs','mediapipe_bridge','licenses','validation','bin']
files=['CMakeLists.txt','README.md','dependencies.lock.json','run_viewer.cmd','run_interaction.cmd',
       'run_parallel_ik.cmd','.gitignore']
paths=[]
for folder in folders:
    paths.extend(p for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
paths.extend(root/p for p in files)
sdk=root/'deps/open3d-devel-windows-amd64-0.19.0'
paths.extend(p for p in sdk.rglob('*') if p.is_file())
paths.extend(root/p for p in ['deps/bazel.exe','deps/mediapipe.zip'])
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as output:
    for path in sorted(paths):
        output.write(path,path.relative_to(root).as_posix())
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
manifest=dict(file=archive.name,sha256=digest,bytes=archive.stat().st_size,files=len(paths))
archive.with_suffix('.zip.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(manifest),flush=True)
