#pragma once
#include <nlohmann/json.hpp>
#include <cstdlib>
namespace general_lab {
using Json=nlohmann::json;
inline Json gates=Json::array();
inline bool active=false;
inline int number(const char* key,int fallback){auto v=std::getenv(key);return v?std::stoi(v):fallback;}
inline bool gate(const char* function,int track,int cluster,const char* name,bool pass,double value,double threshold,int line){
 if(active)gates.push_back({{"function",function},{"track",track},{"cluster",cluster},{"gate",name},{"pass",pass},{"value",value},{"threshold",threshold},{"line",line},{"evaluated",true}});
 return pass;
}
inline const char* reject(int track,int cluster,const char* reason,int line){
 if(active)gates.push_back({{"function","vehicleObservationFailure"},{"track",track},{"cluster",cluster},{"gate","final_observation_reject"},{"reason",reason},{"line",line},{"evaluated",true},{"pass",false},{"final_reject",true}});
 return reason;
}
}
