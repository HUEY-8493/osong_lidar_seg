"""Fail-closed AST function closure for the immutable pre-tracking boundary.
All ambiguous same-name overloads are included. Data declarations are retained;
only function bodies outside the reachable prefix are omitted. No regex body slicing.
"""
from pathlib import Path
import hashlib,json,re,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'.general_replay_python'))
from tree_sitter import Language,Parser
import tree_sitter_cpp
PARSER=Parser(Language(tree_sitter_cpp.language()))
ROOTS=set('filter quantile loadBackground configureVisibility createClusterer BevConnectedComponentClusterer lookupTransform readCloud passValidCrop resetFrameStats updateRangeRingStats applyRoiPolicyToIndices usesStaticBackground classifyForeground radiusOutlierRemoval buildFeaturesFromClusterResults refineVehicleFeaturesWithRawBox estimateVisibilityAngularSpacing publishVisibilityRingProfile prewarmVisibilityRoi separateVehicleUpperAttachments cluster setSensorOrigin configure setPose buildDepth buildGeneralDepth loadBackgroundMap loadRoiPolicy'.split())
def walk(n):
 yield n
 for c in n.children: yield from walk(c)
def sha(b):return hashlib.sha256(b).hexdigest()
def read_source(path):
 text=path.read_text()
 if path.name=='cx128s2_bev_perception_node.cpp':
  text=re.sub(r'#include "general/([^"/]+\.inc\.hpp)"\n\n', lambda m: (path.parent/'general'/m[1]).read_text(), text)
 return text
def signature(pkg):
 files=[*pkg.joinpath('src').glob('*.cpp'),*pkg.joinpath('include').rglob('*.hpp')]
 # Header-only BEV implementation lives in the shared package.
 ws=pkg.parents[1]
 for d in ws.joinpath('src').glob('*/include'):
  if d!=pkg/'include': files.extend(d.rglob('*.hpp'))
 units={};functions={}
 for p in sorted(set(files)):
  if 'crane' in p.name or p.name in ('linear_kalman_filter.hpp','vehicle_status.hpp'):continue
  b=read_source(p).encode();tree=PARSER.parse(b);defs=[]
  for n in walk(tree.root_node):
   if n.type!='function_definition':continue
   dec=n.child_by_field_name('declarator')
   if dec is None:raise RuntimeError('Unrecognized C++ function: '+str(p))
   ids=[x for x in walk(dec) if x.type in ('identifier','field_identifier','destructor_name','operator_name')]
   # Function declarator's own name, not parameter identifiers.
   d=dec
   while d and d.type not in ('identifier','field_identifier','qualified_identifier','destructor_name','operator_name'):
    d=d.child_by_field_name('declarator')
   if d is None:continue
   name=b[d.start_byte:d.end_byte].decode().split('::')[-1]
   body=b[n.start_byte:n.end_byte].decode();defs.append((n,name,body));functions.setdefault(name,[]).append((p,n,body))
  # Macros/preprocessor make the node parse contain benign ERROR nodes. All
  # identified methods are audited in the manifest; unresolved prefix roots fail.
  units[p]=(b,defs)
 missing=ROOTS-functions.keys()
 # Optional loaders differ across versions. Remaining roots must all resolve.
 missing-= {'loadBackgroundMap','loadRoiPolicy'}
 if missing:raise RuntimeError('Unresolved prefix functions: '+str(sorted(missing)))
 reachable=set();pending=list(ROOTS & functions.keys())
 while pending:
  name=pending.pop()
  if name in reachable:continue
  reachable.add(name)
  for _,_,body in functions[name]:
   calls=set(re.findall(r'\b([A-Za-z_]\w*)\s*(?:<[^;{}]*>)?\s*\(',body))
   pending.extend((calls & functions.keys())-reachable)
 # Constructor is not reachable: capture only assignments of members actually
 # consumed by the prefix. This keeps final detection/KF settings independent.
 bodies='\n'.join(body for k in reachable for _,_,body in functions[k])
 node_text=read_source(pkg/'src/cx128s2_bev_perception_node.cpp')
 bodies+='\n'+node_text[node_text.index('  void processCloudFrame('):node_text.index('  // Shared production suffix.')]
 members=set(re.findall(r'\b\w+_\b',bodies));params=set();assignments=[]
 node=pkg/'src/cx128s2_bev_perception_node.cpp';text=read_source(node)
 for match in re.finditer(r'\b(\w+_)\s*=\s*get_parameter\("([^"]+)"\)[^;]*;',text):
  if match[1] in members:params.add(match[2]);assignments.append(match[0])
 params.update(re.findall(r'get_parameter\("([^"]+)"\)',bodies))
 # Visibility initialization, BEV plugin construction and frozen input policy.
 for m in re.finditer(r'(?:declare_parameter|get_parameter)\("([^"]+)"',text):
  if m[1].startswith(('general_ground_noise.','visibility.','cluster','bev_','background_','roi_','raw_vehicle_box_')):params.add(m[1])
 hashes={}
 for p,(b,defs) in units.items():
  # Keep all non-function declarations (including serialized aggregate layouts).
  chunks=[];pos=0
  for n,name,body in sorted(defs,key=lambda x:x[0].start_byte):
   if n.start_byte<pos:continue
   chunks.append(b[pos:n.start_byte]);chunks.append(body.encode() if name in reachable else ('/* downstream '+name+' */').encode());pos=n.end_byte
  chunks.append(b[pos:]);data=b''.join(chunks)
  # Track/runtime declarations are not serialized into the dataset; nevertheless
  # unknown structural changes conservatively invalidate and explain their file.
  hashes[str(p)]=sha(data)
 # Exact prefix control flow (excluding unrelated Crane dispatch and cached restore).
 prefix=text[text.index('  void processCloudFrame('):text.index('  // Shared production suffix.')]
 # Hashing this function is intentional; it contains only front-end control flow.
 hashes['prefix_control']=sha(prefix.encode())
 for _,_,body in functions.get('Cx128s2BevPerception',[]):hashes['constructor_initialization']=sha(body.encode())
 hashes['member_parameter_assignments']=sha('\n'.join(sorted(assignments)).encode())
 return {'version':1,'sources':hashes,'functions':sorted(reachable),'parameters':sorted(params),'codec':sha((pkg/'test/general_replay_cache.hpp').read_bytes()),'extractor':sha(Path(__file__).read_bytes())}
if __name__=='__main__':print(json.dumps(signature(Path(__file__).resolve().parent.parent),indent=2))
