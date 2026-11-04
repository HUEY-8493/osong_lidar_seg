#!/usr/bin/env python3
"""Build the existing include-node replay harness, reusing ROS CMake flags and link objects.
An exact dependency fingerprint avoids recompilation on unchanged repeat invocations.
"""
from pathlib import Path
import argparse, hashlib, json, os, shlex, subprocess, time, fcntl
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--full-only',action='store_true',help='Build Raw full replay without optional AST cache fingerprint dependencies');a=p.parse_args()
test=Path(__file__).resolve().parent;pkg=test.parent;ws=pkg.parent.parent
build=Path(os.environ.get('PERCEPTION_LAB_BUILD_DIR',str(ws/'build/cx128s2_bev_perception')));out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
lock=(out/'build.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
flags={}
for line in (build/'CMakeFiles/test_tracking_policy.dir/flags.make').read_text().splitlines():
 if ' = ' in line:
  k,v=line.split(' = ',1);flags[k]=shlex.split(v)
extra=['-DGENERAL_REPLAY_OFFLINE','-Wno-deprecated-declarations']
for lib in ('rosbag2_cpp','rosbag2_storage','rosbag2_compression','rcpputils'):extra+=['-I/opt/ros/jazzy/include/'+lib]
obj=out/'general_replay.o';exe=out/'general_replay';dep=out/'general_replay.d'
compile=['/usr/bin/c++',*extra,*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-MMD','-MF',str(dep),'-c',str(test/'general_replay.cpp'),'-o',str(obj)]
link=shlex.split((build/'CMakeFiles/test_tracking_policy.dir/link.txt').read_text());link[link.index('CMakeFiles/test_tracking_policy.dir/test/test_tracking_policy.cpp.o')]=str(obj);link[link.index('-o')+1]=str(exe);link+=['-L/opt/ros/jazzy/lib','-lrosbag2_cpp','-lrosbag2_storage','-lcrypto','-llz4','-l:libxxhash.so.0']
# Optional isolated current library overlay; do not mix a current node with stale Crane ABI.
if os.environ.get('PERCEPTION_LAB_LIBRARY_DIR'):
 libdir=Path(os.environ['PERCEPTION_LAB_LIBRARY_DIR']).resolve()
 link=[str(libdir/x) if x.startswith('lib') and x.endswith('.so') and (libdir/x).is_file() else x for x in link]
# Compile current visibility source into an offline object; never accidentally
# test an older workspace libvisibility_model.so after a visibility code edit.
visibility_obj=out/'visibility_model.o'
link=[str(visibility_obj) if Path(x).name=='libvisibility_model.so' else x for x in link]
visibility_compile=['/usr/bin/c++',*extra,*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',str(pkg/'src/visibility_model.cpp'),'-o',str(visibility_obj)]
def dependencies():
 paths=set([pkg/'src/visibility_model.cpp',test/'general_replay.cpp',test/'general_replay_cache.hpp',Path(__file__).resolve()])
 if dep.exists():
  for x in shlex.split(dep.read_text().replace('\\\n',' ').split(':',1)[1]):paths.add(Path(x))
 for x in link:
  q=Path(x)
  if q.is_file() and q not in (obj,exe,visibility_obj):paths.add(q)
 return {str(x.resolve()):hashlib.sha256(x.read_bytes()).hexdigest() for x in sorted(paths) if x.is_file()}
fp=out/'build_fingerprint.json';before={'compile':compile,'visibility_compile':visibility_compile,'link':link,'dependencies':dependencies(),'compiler':subprocess.check_output(['/usr/bin/c++','--version'],text=True)}
t=time.perf_counter();reused=exe.exists() and fp.exists() and json.loads(fp.read_text())==before
if not reused:
 subprocess.run(visibility_compile,check=True);subprocess.run(compile,check=True);subprocess.run(link,cwd=build,check=True)
 after=dependencies()
 changed=[k for k,v in before['dependencies'].items() if k in after and after[k]!=v]
 if changed:
  exe.unlink(missing_ok=True);fp.unlink(missing_ok=True)
  raise RuntimeError('Inputs changed during compilation; rebuild: '+', '.join(changed))
 before['dependencies']=after;fp.write_text(json.dumps(before,indent=2))
# Runtime manifest hashes every package source/header/config plus actual link dependencies.
manifest_paths=set(pkg.joinpath('include').rglob('*.hpp'))|set(pkg.joinpath('src').rglob('*.cpp'))|set(pkg.joinpath('config').glob('*'))|{test/'general_replay.cpp',test/'general_replay_cache.hpp',fp}
manifest_paths|={Path(x) for x in before['dependencies']}
for line in subprocess.check_output(['ldd',str(exe)],text=True).splitlines():
 for word in line.split():
  if word.startswith('/') and Path(word).is_file():manifest_paths.add(Path(word))
manifest_paths|={Path('/usr/include/boost/pfr.hpp'),Path('/usr/include/boost/version.hpp'),Path('/usr/include/nlohmann/json.hpp')}
manifest={str(x.resolve()):hashlib.sha256(x.read_bytes()).hexdigest() for x in sorted(manifest_paths) if x.is_file()}
(out/'runtime_dependencies.json').write_text(json.dumps(manifest,indent=2))
result={'seconds':time.perf_counter()-t,'reused':reused,'executable':str(exe)}
(out/'last_build.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))

if a.full_only:
 prefix_signature={'full_only':True,'cache_compatible':False}
else:
 from dataset_dependencies import signature
 prefix_signature=signature(pkg)
prefix_signature['build_profile']={'compiler':before['compiler'],'defines':flags['CXX_DEFINES'],'includes':flags['CXX_INCLUDES'],'flags':flags['CXX_FLAGS'],'offline_defines':extra}
(out/'prefix_dependencies.json').write_text(json.dumps(prefix_signature,indent=2))
