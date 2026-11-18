#!/usr/bin/env python3
"""Run bounded production HTTP journeys against an isolated ephemeral service."""
import argparse, concurrent.futures, copy, hashlib, json, math, os, pathlib, re, signal, socket, struct, subprocess, tempfile, time, urllib.error, urllib.request, zlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
def decode_png(raw):
    assert raw[:8]==b'\x89PNG\r\n\x1a\n','not a png'
    pos=8;idat=b'';w=h=None
    while pos<len(raw):
        ln,=struct.unpack('>I',raw[pos:pos+4]);typ=raw[pos+4:pos+8];data=raw[pos+8:pos+8+ln];pos+=12+ln
        if typ==b'IHDR':
            w,h,depth,ctype,comp,filt,inter=struct.unpack('>IIBBBBB',data);assert depth==8 and ctype in(2,6)and inter==0,'unexpected png encoding'
        elif typ==b'IDAT':idat+=data
        elif typ==b'IEND':break
    bpp=3 if ctype==2 else 4
    dec=zlib.decompress(idat);stride=w*bpp;out=bytearray();prev=bytearray(stride);p=0
    for y in range(h):
        f=dec[p];p+=1;line=bytearray(dec[p:p+stride]);p+=stride
        if f==1:
            for i in range(bpp,stride):line[i]=(line[i]+line[i-bpp])&255
        elif f==2:
            for i in range(stride):line[i]=(line[i]+prev[i])&255
        elif f==3:
            for i in range(stride):line[i]=(line[i]+(((line[i-bpp] if i>=bpp else 0)+prev[i])>>1))&255
        elif f==4:
            for i in range(stride):
                a=line[i-bpp] if i>=bpp else 0;b=prev[i];c=prev[i-bpp] if i>=bpp else 0;pp=a+b-c
                pa,pb,pc=abs(pp-a),abs(pp-b),abs(pp-c);pr=a if(pa<=pb and pa<=pc)else(b if pb<=pc else c)
                line[i]=(line[i]+pr)&255
        if bpp==3:
            for x in range(w):out+=line[x*3:x*3+3]+b'\xff'
        else:out+=line
        prev=line
    return w,h,out
def px(img,w,x,y):o=(y*w+x)*4;return img[o:o+4]
def bilinear(img,w,h,x,y):
    x0,y0=math.floor(x-.5),math.floor(y-.5);fx,fy=x-.5-x0,y-.5-y0
    def at(xx,yy):return px(img,w,min(max(xx,0),w-1),min(max(yy,0),h-1))
    c00,c10,c01,c11=at(x0,y0),at(x0+1,y0),at(x0,y0+1),at(x0+1,y0+1)
    return tuple(c00[k]*(1-fx)*(1-fy)+c10[k]*fx*(1-fy)+c01[k]*(1-fx)*fy+c11[k]*fx*fy for k in range(4))
# Independent geometric reference: the control-grid warp restated from the
# document contract, without any production code.
def warp_node(spec,i,j):p=spec['points'][j*(spec['cols']+1)+i];return p['x'],p['y']
def warp_apply(spec,x,y):
    ox,oy=spec['origin']['x'],spec['origin']['y'];sx,sy=spec['step']['x'],spec['step']['y'];cols,rows=spec['cols'],spec['rows']
    if not(ox<=x<=ox+cols*sx and oy<=y<=oy+rows*sy):return(x,y)
    fu,fv=(x-ox)/sx,(y-oy)/sy
    ci=min(max(int(math.floor(fu)),0),cols-1);cj=min(max(int(math.floor(fv)),0),rows-1)
    u=min(max(fu-ci,0),1);v=min(max(fv-cj,0),1)
    p00=warp_node(spec,ci,cj);p10=warp_node(spec,ci+1,cj);p01=warp_node(spec,ci,cj+1);p11=warp_node(spec,ci+1,cj+1)
    return((1-u)*(1-v)*p00[0]+u*(1-v)*p10[0]+(1-u)*v*p01[0]+u*v*p11[0],(1-u)*(1-v)*p00[1]+u*(1-v)*p10[1]+(1-u)*v*p01[1]+u*v*p11[1])
def warp_invert(spec,x,y):
    ox,oy=spec['origin']['x'],spec['origin']['y'];sx,sy=spec['step']['x'],spec['step']['y'];cols,rows=spec['cols'],spec['rows']
    if not(ox<=x<=ox+cols*sx and oy<=y<=oy+rows*sy):return(x,y)
    for j in range(rows):
        for i in range(cols):
            q=[warp_node(spec,i,j),warp_node(spec,i+1,j),warp_node(spec,i+1,j+1),warp_node(spec,i,j+1)]
            crosses=[(q[(e+1)%4][0]-q[e][0])*(y-q[e][1])-(q[(e+1)%4][1]-q[e][1])*(x-q[e][0]) for e in range(4)]
            if min(crosses)<-1e-9:continue
            a=(q[1][0]-q[0][0],q[1][1]-q[0][1]);b=(q[3][0]-q[0][0],q[3][1]-q[0][1])
            c=(q[2][0]-q[1][0]-q[3][0]+q[0][0],q[2][1]-q[1][1]-q[3][1]+q[0][1]);d=(x-q[0][0],y-q[0][1])
            a2=a[1]*c[0]-a[0]*c[1];a1=d[0]*c[1]-d[1]*c[0]+a[1]*b[0]-a[0]*b[1];a0=d[0]*b[1]-d[1]*b[0]
            roots=[-a0/a1] if abs(a2)<1e-18 else[(-a1+s)/(2*a2) for s in(math.sqrt(max(a1*a1-4*a2*a0,0)),-math.sqrt(max(a1*a1-4*a2*a0,0)))]
            u=next((r for r in roots if-1e-9<=r<=1+1e-9),0.5);u=min(max(u,0),1)
            dx,dy=b[0]+c[0]*u,b[1]+c[1]*u
            v=(d[0]-a[0]*u)/dx if abs(dx)>=abs(dy) and abs(dx)>1e-18 else(d[1]-a[1]*u)/dy if abs(dy)>1e-18 else 0
            v=min(max(v,0),1)
            return(ox+(i+u)*sx,oy+(j+v)*sy)
    return(x,y)
def components(points,w,h):
    seen=set();count=0
    for start in points:
        if start in seen:continue
        count+=1;stack=[start];seen.add(start)
        while stack:
            x,y=stack.pop()
            for nx,ny in((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                if(nx,ny)in points and(nx,ny)not in seen:seen.add((nx,ny));stack.append((nx,ny))
    return count
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workflow',required=True);args=parser.parse_args()
    seed=json.loads((ROOT/'examples/amber-proof.json').read_text())
    with tempfile.TemporaryDirectory(prefix='prism-plate-') as tmp:
        base=pathlib.Path(tmp);binary=base/'prismplate';subprocess.run(['go','build','-o',str(binary),'./cmd/prismplate'],cwd=ROOT,check=True)
        sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close();url=f'http://127.0.0.1:{port}'
        log=(base/'service.log').open('w');proc=None
        def start():
            nonlocal proc
            proc=subprocess.Popen([str(binary),'-listen',f'127.0.0.1:{port}','-data',str(base/'state')],stdout=log,stderr=log)
            for _ in range(100):
                if proc.poll() is not None:raise RuntimeError('service exited')
                try:
                    urllib.request.urlopen(url+'/healthz',timeout=1).close();return
                except OSError:time.sleep(.05)
            raise RuntimeError('startup timed out')
        def stop():
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGTERM)
                try:proc.wait(timeout=45)
                except subprocess.TimeoutExpired:proc.kill();proc.wait();raise
        def call(method,path,data=None,status=200,raw=False,headers=None):
            body=data if isinstance(data,bytes) else (json.dumps(data).encode() if data is not None else None)
            req=urllib.request.Request(url+path,data=body,method=method,headers=headers or {'Content-Type':'application/json'})
            try:
                with urllib.request.urlopen(req,timeout=35) as r:code=r.status;payload=r.read();hs={k.lower():v for k,v in r.headers.items()}
            except urllib.error.HTTPError as e:code=e.code;payload=e.read();hs={k.lower():v for k,v in e.headers.items()}
            if code!=status:raise RuntimeError(f'{method} {path}: expected {status}, observed {code}: {payload[:200]!r}')
            return (payload,hs) if raw else json.loads(payload) if payload else None
        def render(fmt='png',revision=1):return call('POST',f'/v1/plates/amber-plate/render?revision={revision}&format={fmt}',status=201)
        try:
            start();created=call('POST','/v1/plates',seed,status=201);assert created['revisions'][0]['number']==1
            w=args.workflow
            if w=='create':
                assert call('GET','/v1/plates/amber-plate')['title']==seed['title'];call('POST','/v1/plates',seed,status=409)
                bad=copy.deepcopy(seed);bad['id']='bad-plate';bad['document']['layers'][0]['shapes'][0]['transform']=[0]*6;call('POST','/v1/plates',bad,status=400)
            elif w=='revise':
                q={'expected_revision':1,'document':copy.deepcopy(seed['document'])};q['document']['background']['r']=.2
                with concurrent.futures.ThreadPoolExecutor(2) as pool:
                    def attempt(_):
                        try:return call('PUT','/v1/plates/amber-plate',q)
                        except RuntimeError as e:
                            if 'observed 409' in str(e):return 'conflict'
                            raise
                    results=list(pool.map(attempt,range(2)))
                assert sum(x=='conflict' for x in results)==1;assert len(call('GET','/v1/plates/amber-plate')['revisions'])==2
                assert call('GET','/v1/plates/amber-plate')['revisions'][0]['document']==created['revisions'][0]['document']
            elif w=='branch':
                b=call('POST','/v1/plates/amber-plate/branch',{'revision':1,'id':'amber-fork','title':'琥珀新稿'},status=201);assert b['revisions'][0]['digest']==created['revisions'][0]['digest'];call('POST','/v1/plates/amber-plate/branch',{'revision':99,'id':'missing-fork','title':'无效稿'},status=404)
            elif w=='archive':
                call('POST','/v1/plates/amber-plate/archive');call('PUT','/v1/plates/amber-plate',{'expected_revision':1,'document':seed['document']},status=409);assert render()['revision']==1
            elif w in ('png','svg','artifact'):
                fmt='svg' if w=='svg' else 'png';a=render(fmt);raw,hs=call('GET','/v1/artifacts/'+a['id'],raw=True);assert hashlib.sha256(raw).hexdigest()==a['digest']
                if fmt=='png':assert raw[:8]==b'\x89PNG\r\n\x1a\n';assert struct.unpack('>II',raw[16:24])==(192,144)
                else:assert b'<svg' in raw and b'<linearGradient' in raw and b'clipPath' in raw
                assert render(fmt)['digest']==a['digest'];call('GET','/v1/artifacts/'+a['id'],status=304,raw=True,headers={'If-None-Match':hs['etag']});call('GET','/v1/artifacts/missing',status=404)
                if w=='artifact':call('DELETE','/v1/artifacts/'+a['id']);call('GET','/v1/artifacts/'+a['id'],status=409)
            elif w in ('exposure','cancel'):
                job=call('POST','/v1/plates/amber-plate/exposures?revision=1&format=png',status=202)
                if w=='cancel':call('DELETE','/v1/exposures/'+job['id'],status=202)
                for _ in range(200):
                    job=call('GET','/v1/exposures/'+job['id'])
                    if job['state'] in ('succeeded','cancelled','failed'):break
                    time.sleep(.02)
                assert job['state'] in (('succeeded','cancelled') if w=='cancel' else ('succeeded',))
                if job['state']=='succeeded':assert call('GET','/v1/artifacts/'+job['artifact'],raw=True)[0][:8]==b'\x89PNG\r\n\x1a\n'
                call('DELETE','/v1/exposures/absent',status=404)
            elif w=='proof':
                a=render();call('POST','/v1/proofs',{'id':'proof-amber','artifact':a['id']},status=201);assert call('PUT','/v1/proofs/proof-amber',{'state':'accepted','note':'叠色与镂空检查通过'})['state']=='accepted';call('DELETE','/v1/artifacts/'+a['id'],status=409);call('PUT','/v1/proofs/proof-amber',{'state':'rejected','note':'重复裁决'},status=409)
            elif w in ('bundle','restore'):
                a=render();sealed,_=call('GET','/v1/plates/amber-plate/bundle',raw=True);payload=json.loads(sealed);assert payload['plate']['revisions'][0]['digest']==a['scene_digest']
                if w=='bundle':
                    call('POST','/v1/bundles',sealed,status=409);payload['digest']='0'*64;call('POST','/v1/bundles',payload,status=400)
                    stop();os.rename(base/'state',base/'original');start();imported=call('POST','/v1/bundles',sealed,status=201);assert imported['revisions'][0]['digest']==a['scene_digest'];assert render()['digest']==a['digest']
                else:
                    stop();start();assert call('GET','/v1/artifacts/'+a['id'],raw=True)[0][:8]==b'\x89PNG\r\n\x1a\n';assert render()['digest']==a['digest']
            elif w=='isolation':
                lockproc=subprocess.run([str(binary),'-listen','127.0.0.1:0','-data',str(base/'state')],capture_output=True,timeout=3);assert lockproc.returncode!=0;assert b'locked' in lockproc.stderr
                call('POST','/v1/plates',b'{"id":"x"} true',status=400)
            elif w=='rejection':
                bad=copy.deepcopy(seed);bad['id']='oversize';bad['document']['width']=513;call('POST','/v1/plates',bad,status=400);call('GET','/v1/plates/oversize',status=404)
                bad=copy.deepcopy(seed);bad['id']='invalid-brush';bad['document']['layers'][0]['shapes'][0]['brush']['stops'][1]['at']=0;call('POST','/v1/plates',bad,status=400);assert render()['revision']==1
            elif w=='warp':
                seed2=json.loads((ROOT/'examples/warp-badge.json').read_text())
                plain=copy.deepcopy(seed2)
                for s in plain['document']['layers'][0]['shapes']:s.pop('warp',None)
                created2=call('POST','/v1/plates',plain,status=201)
                def wrender(n,fmt='png'):return call('POST',f'/v1/plates/warp-badge/render?revision={n}&format={fmt}',status=201)
                r0=wrender(1);raw0=call('GET','/v1/artifacts/'+r0['id'],raw=True)[0]
                call('PUT','/v1/plates/warp-badge',{'expected_revision':1,'document':seed2['document']})
                r1=wrender(2);assert wrender(2)['digest']==r1['digest'];raw1=call('GET','/v1/artifacts/'+r1['id'],raw=True)[0]
                canvasdoc=copy.deepcopy(seed2['document'])
                for s in canvasdoc['layers'][0]['shapes']:
                    if 'warp' in s:s['warp']['gradient_space']='canvas'
                call('PUT','/v1/plates/warp-badge',{'expected_revision':2,'document':canvasdoc})
                r2=wrender(3);raw2=call('GET','/v1/artifacts/'+r2['id'],raw=True)[0]
                w0,h0,img0=decode_png(raw0);w1,h1,img1=decode_png(raw1);w2,h2,img2=decode_png(raw2)
                assert(w0,h0)==(200,160)==(w1,h1)==(w2,h2)
                warp=seed2['document']['layers'][0]['shapes'][0]['warp']
                ox,oy=warp['origin']['x'],warp['origin']['y'];gx,gy=ox+warp['cols']*warp['step']['x'],oy+warp['rows']*warp['step']['y']
                for y in range(h0):
                    for x in range(w0):
                        if x+1<=ox or x>=gx or y+1<=oy or y>=gy:assert px(img0,w0,x,y)==px(img1,w1,x,y),f'outside region changed at {x},{y}'
                inside=sum(1 for y in range(50,110) for x in range(110,170) if px(img0,w0,x,y)!=px(img1,w1,x,y))
                assert inside>100,'deformation not visible inside region'
                magenta={(x,y)for y in range(h1)for x in range(w1)if abs(px(img1,w1,x,y)[0]-255)<10 and abs(px(img1,w1,x,y)[1]-51)<10 and abs(px(img1,w1,x,y)[2]-204)<10}
                assert len(magenta)>=8,'landmark lost'
                cx=sum(p[0]+.5 for p in magenta)/len(magenta);cy=sum(p[1]+.5 for p in magenta)/len(magenta)
                ex,ey=warp_apply(seed2['document']['layers'][0]['shapes'][2]['warp'],165,60)
                assert math.hypot(cx-ex,cy-ey)<=1.5,f'landmark at {cx},{cy}, reference {ex},{ey}'
                assert math.hypot(cx-165,cy-60)>2,'landmark did not move'
                green={(x,y)for y in range(h1)for x in range(w1)if abs(px(img1,w1,x,y)[0]-51)<10 and abs(px(img1,w1,x,y)[1]-230)<10 and abs(px(img1,w1,x,y)[2]-140)<10}
                assert len(green)>=150,'strip lost'
                assert components(green,w1,h1)==1,'strip torn across a grid seam'
                qx,qy=150,90
                mx,my=warp_invert(warp,qx+.5,qy+.5)
                exp=bilinear(img0,w0,h0,mx,my);got=px(img1,w1,qx,qy)
                assert max(abs(exp[k]-got[k])for k in range(4))<=6,f'material gradient {got} vs reference {exp}'
                fixed=px(img2,w2,qx,qy);plainpx=px(img0,w0,qx,qy)
                assert max(abs(fixed[k]-plainpx[k])for k in range(4))<=4,f'canvas gradient moved {fixed} vs {plainpx}'
                assert max(abs(int(got[k])-int(fixed[k]))for k in range(4))>=6,'gradient modes secretly share sampling'
                svg2=call('GET','/v1/artifacts/'+wrender(2,'svg')['id'],raw=True)[0].decode()
                svg3=call('GET','/v1/artifacts/'+wrender(3,'svg')['id'],raw=True)[0].decode()
                verts=[float(v)for v in re.findall(r'-?\d+\.?\d*(?:e[+-]?\d+)?',' '.join(re.findall(r'd="([^"]+)"',svg2)))]
                pts=list(zip(verts[0::2],verts[1::2]));wx,wy=warp_apply(warp,160,80)
                assert any(math.hypot(a-wx,b-wy)<1e-6 for a,b in pts),'warped vertex missing from svg'
                g2={k:float(v)for k,v in re.findall(r'(x1|y1|x2|y2)="([^"]+)"',svg2)}
                ax1,ay1=warp_apply(warp,40,80);ax2,ay2=warp_apply(warp,160,80)
                assert abs(g2['x1']-ax1)<1e-6 and abs(g2['y1']-ay1)<1e-6 and abs(g2['x2']-ax2)<1e-6 and abs(g2['y2']-ay2)<1e-6,'material svg gradient axis not warped'
                g3={k:float(v)for k,v in re.findall(r'(x1|y1|x2|y2)="([^"]+)"',svg3)}
                assert(g3['x1'],g3['y1'],g3['x2'],g3['y2'])==(40.0,80.0,160.0,80.0),'canvas svg gradient axis moved'
                def badwarp(edit,needle):
                    doc=copy.deepcopy(seed2['document']);edit(doc['layers'][0]['shapes'][0]['warp']['points'])
                    e=call('PUT','/v1/plates/warp-badge',{'expected_revision':3,'document':doc},status=400)
                    assert needle in e['error'],f'missing {needle!r} in {e["error"]!r}'
                    assert len(call('GET','/v1/plates/warp-badge')['revisions'])==3,'rejected warp changed history'
                badwarp(lambda p:p.__setitem__(12,{'x':141,'y':59}),'folds material')
                badwarp(lambda p:[p.__setitem__(k,v)for k,v in[(6,{'x':100.005,'y':60}),(7,{'x':100.01,'y':60}),(11,{'x':100.005,'y':80}),(12,{'x':100.01,'y':80}),(16,{'x':100.005,'y':100}),(17,{'x':100.01,'y':100})]],'collapses contour')
                badwarp(lambda p:p.__setitem__(10,{'x':98,'y':80}),'pinned')
                badwarp(lambda p:p.__setitem__(12,{'x':185,'y':74}),'grid region')
                kept=call('GET','/v1/plates/warp-badge');assert len(kept['revisions'])==3;assert kept['revisions'][0]['document']==created2['revisions'][0]['document']
                sealed=call('GET','/v1/plates/warp-badge/bundle',raw=True)[0]
                stop();os.rename(base/'state',base/'original');start()
                imported=call('POST','/v1/bundles',sealed,status=201);assert imported['revisions'][1]['digest']==kept['revisions'][1]['digest']
                assert wrender(2)['digest']==r1['digest'],'warped render changed across bundle handover'
            else:raise RuntimeError('unknown workflow '+w)
            print(json.dumps({'workflow':w,'result':'passed','entry':'HTTP','port':port},ensure_ascii=False))
        finally:stop();log.close()
if __name__=='__main__':main()
