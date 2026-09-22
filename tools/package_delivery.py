"""Package the source, runtime, and pinned SDKs, excluding disposable build caches."""
import hashlib
import json
from pathlib import Path
import zipfile

root=Path(__file__).resolve().parents[1]
archive=root.parent/'Mediepipe2Mano_QT-delivery.zip'
folders=['tools','tests','models','assets','configs','mediapipe_bridge','licenses','docs','Bin','vs']
files=['MediaPipe2ManoQt.sln','CMakeLists.txt','CMakePresets.json','CMakeUserPresets.json.example',
       'README.md','dependencies.lock.json','run_viewer.cmd','run_interaction.cmd','run_parallel_ik.cmd','.gitignore',
       'main.cpp','application.cpp','application.h','core.cpp','core.h','detector.cpp','detector.h',
       'skeleton.cpp','skeleton_json.h','worker.h']

def include(path):
    relative=path.relative_to(root)
    if any(part == '__pycache__' for part in relative.parts):
        return False
    if relative.parts[0] == 'vs' and any(
        part in {'x64','Debug','Release','RelWithDebInfo'} for part in relative.parts[1:-1]
    ):
        return False
    if path.name in {'Local.Paths.props','.DS_Store','Thumbs.db'}:
        return False
    if path.name.startswith(('benchmark_','parallelism_benchmark_')) and path.suffix == '.json':
        return False
    if path.name.startswith('README') and '.bak' in path.name:
        return False
    return path.suffix.lower() not in {'.log','.user','.pyc','.pyo'}

paths=[]
for folder in folders:
    paths.extend(p for p in (root/folder).rglob('*')
                 if p.is_file() and include(p))
paths.extend(root/p for p in files)
sdk=root/'ThirdParty/open3d-devel-windows-amd64-0.19.0'
paths.extend(p for p in sdk.rglob('*') if p.is_file())
paths.extend(root/p for p in ['ThirdParty/bazel.exe','ThirdParty/mediapipe.zip'])
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as output:
    for path in sorted(set(paths)):
        output.write(path,path.relative_to(root).as_posix())
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
manifest=dict(file=archive.name,sha256=digest,bytes=archive.stat().st_size,files=len(set(paths)))
archive.with_suffix('.zip.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(manifest),flush=True)
