#!/usr/bin/env python3
"""Build an inspection-only persistent adapter around the unchanged Replay reader."""
from pathlib import Path
import argparse,fcntl,sys
here=Path(__file__).resolve().parent;test=here.parent;sys.path.insert(0,str(test))
p=argparse.ArgumentParser();p.add_argument('--state',type=Path,required=True);a=p.parse_args();out=(a.state/'studio_reader').resolve();out.mkdir(parents=True,exist_ok=True)
lock=(out/'generator.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
s=(a.state/'tool/general_debug_replay.cpp').read_text()
anchor=' TrackingRegressionAccess::run(*n,mode,bag,cache,out,'
assert s.count(anchor)==1
adapter=''' if(mode=="studio-reader") {
  TrackingRegressionAccess::stop(*n);
  std::string line;
  while(std::getline(std::cin,line)) {
   Json response;
   try {
    auto request=Json::parse(line);
    const auto target=request.at("frame").get<uint64_t>();
    if(target==0)throw std::runtime_error("Viewer frame must be positive");
    TrackingRegressionAccess::run(*n,"cache-replay",request.at("bag").get<std::string>(),
      request.at("cache").get<std::string>(),request.at("prefix").get<std::string>(),
      fs::canonical("/proc/self/exe"),{},request.at("depth_cache").get<bool>(),false,
      request.at("compressed").get<bool>(),true,0,target);
    response={{"ok",true}};
   }catch(const std::exception&e){response={{"ok",false},{"error",e.what()}};}
   std::cout<<response.dump()<<std::endl;
  }
  n.reset();rclcpp::shutdown();return 0;
 }
'''
s=s.replace(anchor,adapter+anchor)
source=out/'studio_reader.cpp'
if not source.exists() or source.read_text()!=s:source.write_text(s)
script=(test/'build_replay.py').read_text().replace("test/'general_replay.cpp'",'Path('+repr(str(source))+')')
sys.argv=[str(test/'build_replay.py'),'--output',str(out)]
exec(compile(script,str(test/'build_replay.py'),'exec'),{'__name__':'__main__','__file__':str(test/'build_replay.py')})
