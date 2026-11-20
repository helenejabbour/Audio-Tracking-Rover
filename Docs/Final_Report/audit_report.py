"""Source-integrity and mathematical spot checks for the final-report draft."""
from pathlib import Path
from datetime import datetime, timezone
from collections import Counter
import re
import json
import hashlib
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

def expand(path):
    s = path.read_text(encoding='utf-8')
    return re.sub(r'\\input\{([^}]+)\}', lambda m: expand(HERE/(m[1]+'.tex')), s)

text = expand(HERE/'main.tex')
# Ignore comments for delimiter/reference checks, retaining escaped percent signs.
clean = '\n'.join(re.split(r'(?<!\\)%', line)[0] for line in text.splitlines())
labels = re.findall(r'\\label\{([^}]+)\}', clean)
refs = re.findall(r'\\(?:ref|eqref|autoref)\{([^}]+)\}', clean)
bib = (HERE/'refs.bib').read_text(encoding='utf-8')
keys = re.findall(r'@\w+\s*\{\s*([^,]+),', bib)
cites = [k.strip() for v in re.findall(r'\\cite\w*(?:\[[^]]*\])*\{([^}]+)\}',clean) for k in v.split(',')]
stack=[]; errors=[]
for match in re.finditer(r'\\(begin|end)\{([^}]+)\}',clean):
    if match[1]=='begin': stack.append(match[2])
    elif not stack or stack.pop()!=match[2]: errors.append('Environment mismatch: '+match[0])
if stack: errors.append('Unclosed environments: '+str(stack))
depth=0
for match in re.finditer(r'(?<!\\)[{}]',clean):
    depth += 1 if match[0]=='{' else -1
    if depth<0: errors.append('Negative brace nesting');break
if depth: errors.append('Nonzero brace balance: '+str(depth))
images=re.findall(r'\\includegraphics(?:\[[^]]*\])?\{([^}]+)\}',clean)
preserved=json.loads((HERE/'audit/provenance.json').read_text())
hashcheck={name:hashlib.sha256((HERE/'sections'/name).read_text(encoding='utf-8').encode()).hexdigest()==digest
           for name,digest in preserved['preserved_blocks_normalized_lf_sha256'].items()}

# Compare the measurement-inversion Jacobian against independent finite differences.
rho,beta,center,sigma,delta,n,K=1.7,.13,-.04,.7,.3,1.2,100.
ell=np.log([K/rho**n*np.exp(-(center+d-beta)**2/(2*sigma**2)) for d in [delta,-delta]])
def inv(l):
    b=center+sigma**2/(2*delta)*(l[0]-l[1])
    r=np.exp((np.log(K)-np.mean(l)-((center-b)**2+delta**2)/(2*sigma**2))/n)
    return np.array([r,b])
J=np.array([[rho/n*(-.5+(center-beta)/(2*delta)),rho/n*(-.5-(center-beta)/(2*delta))],
            [sigma*sigma/(2*delta),-sigma*sigma/(2*delta)]])
h=1e-6
numeric=np.column_stack([(inv(ell+np.eye(2)[i]*h)-inv(ell-np.eye(2)[i]*h))/(2*h) for i in range(2)])
a=.15;Ab=np.array([[0,a],[0,0]]);Bb=np.array([[ -1.],[-1.]])
Tb=np.array([[-a,-1],[0,-1]])
canonical_error=float(np.max(abs(np.linalg.solve(Tb,Ab@Tb)-np.array([[0,1],[0,0]]))))
source_snapshot=ROOT/'Runs/2026-11-15/20261115T163143_844160Z_validation/source_snapshot'
snapshot_differences=[]
for p in (ROOT/'rover').glob('*.py'):
    snap=source_snapshot/'rover'/p.name
    if not snap.exists() or p.read_bytes()!=snap.read_bytes():snapshot_differences.append(p.name)

protected=set(preserved['preserved_blocks_normalized_lf_sha256'])
banned=r'\b(delve|tapestry|realm|pivotal|testament|fostering|beacon|crucial|paramount|groundbreaking|underscores|transformative|furthermore|moreover)\b'
style_hits={p.name:re.findall(banned,p.read_text(encoding='utf-8'),re.I)
            for p in [HERE/'main.tex',*(HERE/'sections').glob('*.tex')] if p.name not in protected}
style_hits={k:v for k,v in style_hits.items() if v}
report={
 'checked_utc':datetime.now(timezone.utc).isoformat(),
 'missing_references':sorted(set(refs)-set(labels)),
 'duplicate_labels':[k for k,v in Counter(labels).items() if v>1],
 'missing_bibliography_keys':sorted(set(cites)-set(keys)),
 'duplicate_bibliography_keys':[k for k,v in Counter(keys).items() if v>1],
 'missing_images':[name for name in images if not (HERE/name).exists()],
 'syntax_errors':errors,
 'preserved_block_hash_checks':hashcheck,
 'bibliography_byte_identical':(HERE/'refs.bib').read_bytes()==(ROOT/'Docs/Audio_Tracking_Rover_Progress_Report/refs.bib').read_bytes(),
 'new_text_banned_word_hits':style_hits,
 'measurement_jacobian_max_error':float(np.max(abs(J-numeric))),
 'canonical_A_error':canonical_error,
 'canonical_B_error':float(np.max(abs(np.linalg.solve(Tb,Bb)-np.array([[0],[1]])))),
 'runtime_source_differences_from_saved_validation':snapshot_differences,
 'red_placeholder_count':len(re.findall(r'\\pending\{',clean))+len(re.findall(r'\\textcolor\{red\}\{\[TO COMPLETE:',clean)),
 'compile_status':'BLOCKED: uncached Tectonic bundle inaccessible; MiKTeX initialization/registry denied',
 'visual_layout_verified':False,
}
bad=any(report[k] for k in ['missing_references','duplicate_labels','missing_bibliography_keys','duplicate_bibliography_keys','missing_images','syntax_errors','new_text_banned_word_hits','runtime_source_differences_from_saved_validation'])
report['source_checks_passed']=not bad and all(hashcheck.values()) and report['measurement_jacobian_max_error']<1e-6
(HERE/'audit/source_checks.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
if not report['source_checks_passed']:raise SystemExit(1)
