/* Offline browser reconstruction of measured Isaac poses. No physics is performed. */
(() => {
  'use strict';
  const DATA = JSON.parse(document.getElementById('pose-data').textContent);
  const $ = id => document.getElementById(id);
  const names = {dinner_plate:'Dinner plate', bowl:'Bowl', mug:'Mug'};
  const racks = {LowerRack:'Lower rack', UpperRack:'Upper rack'};
  const colors = {dinner_plate:0x2d8290, bowl:0xd78b54, mug:0x7778bb};
  const state = {filtered:[], selected:null, stage:'final', playing:false, view:'iso'};
  let playTimer, renderer, thumbRenderer, mainView, thumbView, camera, thumbCamera;
  let selectedGroup, cloud, observer, queue = [], generation = 0, processing = false;
  const thumbnailCache = new Map();
  const geometry = {}, materials = {};
  const orbit = {target:new THREE.Vector3(0,0,.45), radius:1.3, azimuth:-1.02, elevation:.62};
  let selectedBounds = null;
  const fmt = (n, digits=6) => Number.isFinite(n) ? (Math.abs(n)<.5*10**-digits ? 0 : n).toFixed(digits) : '—';
  const mm = n => Number.isFinite(n) ? fmt(n*1000,3)+' mm' : 'Unavailable';
  const el = (tag, cls, text) => {const node=document.createElement(tag);if(cls)node.className=cls;if(text!==undefined)node.textContent=text;return node;};
  const effective = (trial, requested=state.stage) => trial[requested] ? requested : null;
  function stageName(stage, trial) {
    if(!stage) return 'No '+state.stage+' pose recorded';
    if(stage==='sampled') return trial.sampled.released_into_physics ? 'Sampled · release pose' : 'Sampled · rejected before release';
    if(stage==='settled') return trial.unresolved || trial.outcome==='settle_timeout' ? 'Settling snapshot' : 'Settled · before retraction';
    return trial.outcome==='accepted' ? 'Final · accepted placement' : 'Final snapshot · not accepted';
  }
  function euler(pose) {
    const q=new THREE.Quaternion().fromArray(pose.quaternion_xyzw).normalize();
    const e=new THREE.Euler().setFromQuaternion(q,'XYZ');
    return [e.x,e.y,e.z].map(v=>v*180/Math.PI);
  }
  function resultLabel(t) {return t.outcome==='accepted' ? t.strict ? 'Strictly inside' : '1 mm tolerance' : t.unresolved ? 'Unresolved' : t.outcome==='initial_collision'?'Never dropped':t.outcome.replaceAll('_',' ');}
  function accepts(t) {
    const kind=$('kind-filter').value,rack=$('rack-filter').value,result=$('outcome-filter').value,search=$('search').value.trim().toLowerCase();
    return (kind==='all'||t.kind===kind)&&(rack==='all'||t.rack===rack)&&(!search||t.id.toLowerCase().includes(search))&&
      (result==='all'||result==='accepted'&&t.outcome==='accepted'||result==='strict'&&t.strict===true||
       result==='tolerance'&&t.outcome==='accepted'&&t.strict===false||result==='unresolved'&&t.unresolved||
       result==='rejected'&&t.outcome!=='accepted'&&!t.unresolved);
  }
  function filter() {
    stopPlayback();
    state.filtered=DATA.trials.filter(accepts);
    const selected=state.filtered.find(t=>t.id===state.selected?.id)||state.filtered[0]||null;
    $('filtered-count').textContent=state.filtered.length;
    $('empty-state').hidden=state.filtered.length!==0;
    $('pose-slider').max=Math.max(0,state.filtered.length-1);
    $('pose-slider').disabled=!state.filtered.length;
    ['prev-pose','next-pose','play-poses','export-filtered'].forEach(id=>$(id).disabled=!state.filtered.length);
    buildGallery();select(selected,true);
  }
  function resetFilters() {
    $('kind-filter').value='all';$('rack-filter').value='all';$('outcome-filter').value='accepted';$('search').value='';filter();
  }
  function vector(container, labels, values, digits) {
    container.replaceChildren(...labels.map((label,i)=>{const box=el('div','value-cell');box.append(el('span','',label),el('code','',values ? fmt(values[i],digits):'—'));return box;}));
  }
  function metric(label,value) {const row=el('div');row.append(el('dt','',label),el('dd','',value));return row;}
  function clearMainPose(){selectedBounds=null;if(mainView){clearObjects(mainView);if(cloud){mainView.scene.remove(cloud);cloud.geometry.dispose();cloud.material.dispose();cloud=null;}render();}}
  function select(trial,fit=false) {
    const old=state.selected;state.selected=trial;
    document.querySelectorAll('[data-stage]').forEach(b=>{b.classList.toggle('active',b.dataset.stage===state.stage);b.setAttribute('aria-pressed',String(b.dataset.stage===state.stage));});
    document.querySelector('.pose-card.selected')?.classList.remove('selected');
    document.querySelectorAll('.pose-card[aria-pressed="true"]').forEach(n=>n.setAttribute('aria-pressed','false'));
    const card=trial&&document.getElementById('card-'+trial.id);if(card){card.classList.add('selected');card.setAttribute('aria-pressed','true');}
    const i=state.filtered.indexOf(trial);$('pose-slider').value=Math.max(0,i);$('pose-progress').textContent=(i+1)+' / '+state.filtered.length;
    $('export-pose').disabled=!trial;
    $('stage-missing').hidden=true;
    $('viewport').classList.remove('rejected-proposal');
    if(!trial){
      $('trial-id').textContent='No pose selected';$('result-badge').textContent='—';$('result-description').textContent='Change the filters to explore more poses.';
      $('scene-kind').textContent='No matching poses';$('scene-rack').textContent='';$('stage-caption').textContent='';$('stage-notice').hidden=true;
      vector($('position-values'),['X','Y','Z'],null,6);vector($('euler-values'),['X','Y','Z'],null,2);vector($('quaternion-values'),['X','Y','Z','W'],null,6);$('pose-metrics').replaceChildren();
      clearMainPose();return;
    }
    const stage=effective(trial),record=stage?trial[stage]:null,pose=record?.dish;
    $('trial-id').textContent=trial.id;$('result-badge').textContent=trial.outcome==='accepted'?'Accepted':trial.unresolved?'Unresolved':'Rejected';
    $('result-badge').className='badge'+(trial.outcome==='accepted'?'':trial.unresolved?' warning':' rejected');
    $('result-description').textContent=trial.outcome==='accepted' ? trial.strict ? 'Final placement: rack closure passed, with the entire object strictly inside the enclosure in both audited frames.' : 'Final placement: rack closure passed using the configured 1 mm boundary tolerance.' : trial.outcome==='initial_collision'?'Rejected before drop: the proposed bowl or dish intersects the modeled dishwasher. No physical release, settling, or loaded rack-closure test occurred.':trial.reason;
    $('scene-kind').textContent=names[trial.kind];$('scene-rack').textContent=racks[trial.rack]+' · trial '+String(trial.index).padStart(3,'0');
    $('stage-caption').textContent=stageName(stage,trial);
    const notices=[];
    if(stage==='sampled'&&!trial.sampled.released_into_physics)notices.push('REJECTED PROPOSAL — NEVER DROPPED. This red ghost shows a colliding candidate, not a simulated placement.');
    else if(trial.outcome!=='accepted'&&stage)notices.push(trial.unresolved?'Numerically unresolved; this is not a validated placement.':'This trial did not pass acceptance.');
    if(record?.endpoint_matches_scene_snapshot===false)notices.push('Showing the last synchronized scene snapshot; a later dish endpoint is included separately in the export.');
    $('stage-notice').textContent=notices.join(' ');$('stage-notice').hidden=!notices.length;
    vector($('position-values'),['X','Y','Z'],pose?.position_m,6);vector($('euler-values'),['X','Y','Z'],pose?euler(pose):null,2);vector($('quaternion-values'),['X','Y','Z','W'],pose?.quaternion_xyzw,6);
    $('pose-metrics').replaceChildren(
      metric('Displayed stage',!stage?'Unavailable':stage==='sampled'?'Sampled':stage==='settled'?'Settling snapshot':'Final snapshot'),
      metric('Physical drop',trial.sampled.released_into_physics?'Performed':'Never performed'),
      metric('Rack retraction',trial.closure_attempted?'Attempted':'Not started'),
      metric('Final endpoint error',mm(trial.metrics.final_endpoint_error_m)),
      metric('Final world boundary clearance',mm(trial.metrics.world_clearance_m)),
      metric('Retraction / hold penetration',mm(trial.metrics.maximum_cycle_penetration_m)),
      metric('Settled peak penetration',mm(trial.metrics.settled_peak_penetration_m)));
    const onlySampled=!stage||stage==='sampled';
    $('show-other-rack').disabled=onlySampled;$('show-shell').disabled=onlySampled;
    if(!stage){
      $('stage-missing').hidden=false;
      $('missing-title').textContent='No '+state.stage+' pose exists for this trial';
      $('missing-description').textContent=trial.outcome==='initial_collision'?'The initial collision check rejected this proposal. The bowl or dish was never dropped, so there is no simulated endpoint to display.':'The trial stopped before this stage was recorded. No earlier pose has been substituted.';
      $('show-available-pose').textContent=trial.settled?'View settling snapshot':trial.outcome==='initial_collision'?'View rejected proposal':'View starting proposal';
      $('show-available-pose').dataset.stageTarget=trial.settled?'settled':'sampled';
      clearMainPose();
    }else if(mainView){
      selectedBounds=populate(mainView,trial,stage,false);
      updateCloud();
      if(fit||!old||old.rack!==trial.rack||effective(old)!==stage)fitCamera();else render();
    }
    $('viewport').classList.toggle('rejected-proposal',stage==='sampled'&&!trial.sampled.released_into_physics);
    const n=state.filtered.filter(t=>t[state.stage]).length;
    $('gallery-description').textContent=state.stage==='sampled'?'Starting proposals only. Red ghosts were rejected before any drop.':n+' '+state.stage+' scenes available; missing stages show empty cards.';
  }
  function step(delta){if(!state.filtered.length)return;const n=state.filtered.length,i=(state.filtered.indexOf(state.selected)+delta+n)%n;select(state.filtered[i]);}
  function stopPlayback(){clearInterval(playTimer);state.playing=false;$('play-poses').textContent='▶';$('play-poses').setAttribute('aria-label','Play through recorded poses');}
  function play(){if(state.playing){stopPlayback();return;}if(!state.filtered.length)return;state.playing=true;$('play-poses').textContent='Ⅱ';$('play-poses').setAttribute('aria-label','Pause recorded poses');playTimer=setInterval(()=>step(1),1800);}
  function exportData(payload,filename){const blob=new Blob([JSON.stringify(payload,null,2)+'\n'],{type:'application/json'});const url=URL.createObjectURL(blob),a=el('a');a.href=url;a.download=filename;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  function exportRecord(t){const stage=effective(t);return {run_id:DATA.meta.run_id,units:'metres',up_axis:'Z',quaternion_order:'XYZW',requested_stage:state.stage,displayed_stage:stage,displayed_pose:stage?t[stage].dish:null,trial:t};}
  function buildGallery(){
    generation++;queue=[];observer?.disconnect();const gen=generation;
    const fragment=document.createDocumentFragment();
    for(const t of state.filtered){
      const stage=effective(t),card=el('button','pose-card');card.id='card-'+t.id;card.type='button';card.dataset.trialId=t.id;card.setAttribute('aria-pressed','false');
      card.setAttribute('aria-label',`${names[t.kind]}, ${racks[t.rack]}, trial ${t.index}, ${resultLabel(t)}, ${stageName(stage,t)}`);
      const visual=el('div','card-image'),image=el('img');image.alt=`${names[t.kind]} ${stage||'unavailable'} pose for ${t.id}`;image.hidden=true;
      card.dataset.hasStage=String(!!stage);
      if(!stage)visual.classList.add('missing-image');
      if(stage==='sampled'&&!t.sampled.released_into_physics)visual.classList.add('rejected-image');
      visual.append(image,el('span',stage?'placeholder':'missing-label',stage?'Loading pose…':'No '+state.stage+' pose'),el('span','card-stage',!stage?'No simulation at this stage':stage==='sampled'&&!t.sampled.released_into_physics?'Rejected proposal · never dropped':stage==='sampled'?'Sampled':stage==='settled'?'Settling snapshot':'Final'));
      const info=el('div','card-info'),top=el('div','card-top');top.append(el('span','card-kind',names[t.kind]),el('span','card-index','#'+String(t.index).padStart(3,'0')));
      const bottom=el('div','card-bottom');const degrees=stage?euler(t[stage].dish).map(x=>fmt(x,0)+'°').join(' / '):'—';
      bottom.append(el('span','card-result'+(t.outcome==='accepted'?'':' fail'),resultLabel(t)),el('span','card-rotation',degrees));
      info.append(top,el('div','card-rack',racks[t.rack]),bottom);card.append(visual,info);card.addEventListener('click',()=>{stopPlayback();select(t);});fragment.append(card);
    }
    $('pose-gallery').replaceChildren(fragment);
    if(!thumbRenderer){$('pose-gallery').querySelectorAll('.placeholder').forEach(p=>p.textContent='3D unavailable');return;}
    observer=new IntersectionObserver(entries=>{for(const entry of entries)if(entry.isIntersecting){observer.unobserve(entry.target);queue.push({card:entry.target,gen});}processQueue();},{rootMargin:'350px'});
    $('pose-gallery').querySelectorAll('.pose-card[data-has-stage="true"]').forEach(card=>observer.observe(card));
  }
  function processQueue(){
    if(processing||!queue.length)return;processing=true;
    requestAnimationFrame(()=>{
      const job=queue.shift();
      try{if(job&&job.gen===generation&&job.card.isConnected){const t=DATA.trials.find(t=>t.id===job.card.dataset.trialId),stage=effective(t),key=t.id+'-'+stage;let src=thumbnailCache.get(key);
        if(!src){const bounds=populate(thumbView,t,stage,true),center=bounds.getCenter(new THREE.Vector3());const radius=fitDistance(bounds,thumbCamera,1.06);
          thumbCamera.position.copy(center).add(new THREE.Vector3(.65,-1,.8).normalize().multiplyScalar(radius));thumbCamera.lookAt(center);thumbRenderer.render(thumbView.scene,thumbCamera);src=thumbRenderer.domElement.toDataURL('image/webp',.78);thumbnailCache.set(key,src);}
        const img=job.card.querySelector('img');img.src=src;img.hidden=false;job.card.querySelector('.placeholder')?.remove();
      }}catch(error){console.error('Thumbnail render failed',error);job?.card.querySelector('.placeholder')?.replaceChildren(document.createTextNode('Preview unavailable'));}
      processing=false;if(queue.length)processQueue();
    });
  }
  function makeView(){
    const scene=new THREE.Scene();scene.background=new THREE.Color(0xf0f3f0);
    scene.add(new THREE.HemisphereLight(0xffffff,0xadb9af,2.0));
    const light=new THREE.DirectionalLight(0xffffff,2.2);light.position.set(1,-1,2);scene.add(light);
    const fill=new THREE.DirectionalLight(0xffffff,.8);fill.position.set(-1,1,.8);scene.add(fill);
    const objects=new THREE.Group();scene.add(objects);return {scene,objects};
  }
  function posedMesh(name,pose,material){const mesh=new THREE.Mesh(geometry[name],material);mesh.position.fromArray(pose.position_m);mesh.quaternion.fromArray(pose.quaternion_xyzw).normalize();return mesh;}
  function clearObjects(view){view.objects.children.forEach(child=>{if(child.type==='AxesHelper'){child.geometry.dispose();child.material.dispose();}});view.objects.clear();}
  function populate(view,trial,stage,thumbnail){
    clearObjects(view);const record=trial[stage],frames=record.frames;
    const rack=posedMesh(trial.rack,frames[trial.rack],materials.rack);view.objects.add(rack);
    if(frames.SilverwareBasket&&trial.rack==='LowerRack')view.objects.add(posedMesh('SilverwareBasket',frames.SilverwareBasket,materials.basket));
    const dish=posedMesh(trial.kind,record.dish,stage==='sampled'&&!trial.sampled.released_into_physics?materials.rejected:materials[trial.kind]);view.objects.add(dish);
    const bounds=new THREE.Box3().setFromObject(rack);bounds.expandByObject(dish);
    if(!thumbnail){
      const other=trial.rack==='LowerRack'?'UpperRack':'LowerRack';
      if($('show-other-rack').checked&&frames[other])view.objects.add(posedMesh(other,frames[other],materials.otherRack));
      if($('show-shell').checked&&frames.Cabinet)view.objects.add(posedMesh('Cabinet',frames.Cabinet,materials.shell));
      if($('show-axes').checked){const axes=new THREE.AxesHelper(.105);axes.position.copy(dish.position);axes.quaternion.copy(dish.quaternion);axes.material.depthTest=false;axes.renderOrder=10;view.objects.add(axes);}
    }
    view.objects.updateMatrixWorld(true);return bounds;
  }
  function initScene(){
    try{
      renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,preserveDrawingBuffer:false});renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
      $('viewport').prepend(renderer.domElement);renderer.domElement.setAttribute('aria-label','Exact dish and rack geometry');
      for(const [name,mesh]of Object.entries(DATA.meshes)){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(mesh.positions,3));g.setIndex(mesh.indices);g.computeVertexNormals();g.computeBoundingBox();geometry[name]=g;}
      for(const [kind,color]of Object.entries(colors))materials[kind]=new THREE.MeshStandardMaterial({color,roughness:.32,metalness:.06,side:THREE.DoubleSide});
      materials.rejected=new THREE.MeshStandardMaterial({color:0xc14d4d,roughness:.6,transparent:true,opacity:.40,depthWrite:false,side:THREE.DoubleSide});
      materials.rack=new THREE.MeshStandardMaterial({color:0x9fac9f,roughness:.7,metalness:.15,transparent:true,opacity:.38,depthWrite:false,side:THREE.DoubleSide});
      materials.otherRack=new THREE.MeshStandardMaterial({color:0xa6b1a7,roughness:.8,transparent:true,opacity:.11,depthWrite:false,side:THREE.DoubleSide});
      materials.basket=new THREE.MeshStandardMaterial({color:0x829485,roughness:.8,transparent:true,opacity:.15,depthWrite:false,side:THREE.DoubleSide});
      materials.shell=new THREE.MeshStandardMaterial({color:0xa8b8af,roughness:.8,transparent:true,opacity:.055,depthWrite:false,side:THREE.DoubleSide});
      mainView=makeView();camera=new THREE.PerspectiveCamera(37,1,.005,30);camera.up.set(0,0,1);
      const envelope=new THREE.Box3(new THREE.Vector3().fromArray(DATA.bounds.min),new THREE.Vector3().fromArray(DATA.bounds.max));
      const outline=new THREE.Box3Helper(envelope,0xc7d2c8);outline.material.transparent=true;outline.material.opacity=.45;mainView.scene.add(outline);
      const grid=new THREE.GridHelper(3,30,0xd5ddd3,0xe1e7de);grid.rotation.x=Math.PI/2;grid.position.z=-.012;mainView.scene.add(grid);
      thumbRenderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});thumbRenderer.setSize(320,210);thumbRenderer.outputColorSpace=THREE.SRGBColorSpace;thumbView=makeView();thumbCamera=new THREE.PerspectiveCamera(36,320/210,.005,20);thumbCamera.up.set(0,0,1);
      new ResizeObserver(()=>{const w=$('viewport').clientWidth,h=$('viewport').clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();if(selectedBounds)fitCamera();else render();}).observe($('viewport'));
      bindCamera(renderer.domElement);
    }catch(error){console.warn('WebGL unavailable:',error);$('webgl-error').hidden=false;renderer?.dispose();renderer=null;mainView=null;thumbRenderer=null;}
  }
  function render(){if(!renderer||!camera)return;camera.position.copy(orbit.target).add(new THREE.Vector3(Math.cos(orbit.azimuth)*Math.cos(orbit.elevation),Math.sin(orbit.azimuth)*Math.cos(orbit.elevation),Math.sin(orbit.elevation)).multiplyScalar(orbit.radius));camera.lookAt(orbit.target);renderer.render(mainView.scene,camera);}
  function fitDistance(box,viewCamera,padding=1.12){const halfVertical=THREE.MathUtils.degToRad(viewCamera.fov/2),halfHorizontal=Math.atan(Math.tan(halfVertical)*viewCamera.aspect);return Math.max(.4,box.getSize(new THREE.Vector3()).length()/2/Math.sin(Math.min(halfVertical,halfHorizontal))*padding);}
  function fitCamera(){if(!selectedBounds)return;const box=selectedBounds.clone();if($('show-cloud').checked&&cloud)box.expandByObject(cloud);box.getCenter(orbit.target);orbit.radius=fitDistance(box,camera);render();}
  function viewPreset(name){state.view=name;if(name==='front'){orbit.azimuth=-Math.PI/2;orbit.elevation=.04;}else if(name==='side'){orbit.azimuth=0;orbit.elevation=.04;}else if(name==='top'){orbit.azimuth=-Math.PI/2;orbit.elevation=Math.PI/2-.001;}else{orbit.azimuth=-1.02;orbit.elevation=.62;}
    document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));render();}
  function bindCamera(canvas){
    const pointers=new Map();let lastPinch=null;
    canvas.addEventListener('contextmenu',e=>e.preventDefault());
    canvas.addEventListener('pointerdown',e=>{pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});canvas.setPointerCapture(e.pointerId);lastPinch=null;});
    canvas.addEventListener('pointermove',e=>{const previous=pointers.get(e.pointerId);if(!previous)return;const dx=e.clientX-previous.x,dy=e.clientY-previous.y;pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});
      if(pointers.size===2){const[a,b]=[...pointers.values()],distance=Math.hypot(a.x-b.x,a.y-b.y);if(lastPinch)orbit.radius=Math.max(.12,Math.min(8,orbit.radius*lastPinch/distance));lastPinch=distance;}
      else if(e.buttons===2||e.shiftKey){const right=new THREE.Vector3().setFromMatrixColumn(camera.matrix,0),up=new THREE.Vector3().setFromMatrixColumn(camera.matrix,1);orbit.target.addScaledVector(right,-dx*orbit.radius*.0015).addScaledVector(up,dy*orbit.radius*.0015);}
      else{orbit.azimuth-=dx*.006;orbit.elevation=Math.max(-1.45,Math.min(1.56,orbit.elevation+dy*.006));document.querySelectorAll('[data-view]').forEach(b=>b.classList.remove('active'));}
      render();});
    const end=e=>{pointers.delete(e.pointerId);lastPinch=null;};canvas.addEventListener('pointerup',end);canvas.addEventListener('pointercancel',end);
    canvas.addEventListener('wheel',e=>{e.preventDefault();orbit.radius=Math.max(.12,Math.min(8,orbit.radius*Math.exp(e.deltaY*.001)));render();},{passive:false});
  }
  function updateCloud(){
    if(cloud){mainView.scene.remove(cloud);cloud.geometry.dispose();cloud.material.dispose();cloud=null;}
    if(!$('show-cloud').checked)return;
    const positions=[],rgb=[];for(const t of state.filtered){if(!t[state.stage])continue;positions.push(...t[state.stage].dish.position_m);const c=new THREE.Color(colors[t.kind]);rgb.push(c.r,c.g,c.b);}
    const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));g.setAttribute('color',new THREE.Float32BufferAttribute(rgb,3));
    cloud=new THREE.Points(g,new THREE.PointsMaterial({size:.011,vertexColors:true,transparent:true,opacity:.7,depthTest:false}));mainView.scene.add(cloud);
  }
  $('total-count').textContent=DATA.trials.length;$('accepted-count').textContent=DATA.trials.filter(t=>t.outcome==='accepted').length;$('strict-count').textContent=DATA.trials.filter(t=>t.strict).length;
  $('footer-run').textContent=DATA.meta.run_id;
  $('provenance').textContent='Run '+DATA.meta.run_id+' · seed '+DATA.meta.seed+' · completed '+DATA.meta.experiment_finished_utc.slice(0,10)+'. Accepted evidence and geometry audits: '+DATA.meta.audits.accepted_evidence+' / '+DATA.meta.audits.accepted_geometry+'.';
  ['kind-filter','rack-filter','outcome-filter'].forEach(id=>$(id).addEventListener('change',filter));$('search').addEventListener('input',filter);
  $('reset-filters').addEventListener('click',resetFilters);$('empty-reset').addEventListener('click',resetFilters);
  document.querySelectorAll('[data-stage]').forEach(b=>b.addEventListener('click',()=>{stopPlayback();state.stage=b.dataset.stage;buildGallery();select(state.selected,true);}));
  $('show-available-pose').addEventListener('click',()=>{stopPlayback();state.stage=$('show-available-pose').dataset.stageTarget;buildGallery();select(state.selected,true);});
  document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>viewPreset(b.dataset.view)));$('fit-camera').addEventListener('click',fitCamera);
  ['show-other-rack','show-shell','show-axes','show-cloud'].forEach(id=>$(id).addEventListener('change',()=>select(state.selected,id==='show-cloud')));
  $('prev-pose').addEventListener('click',()=>{stopPlayback();step(-1);});$('next-pose').addEventListener('click',()=>{stopPlayback();step(1);});$('play-poses').addEventListener('click',play);
  $('pose-slider').addEventListener('input',()=>{stopPlayback();select(state.filtered[Number($('pose-slider').value)]);});
  $('export-pose').addEventListener('click',()=>{if(state.selected)exportData(exportRecord(state.selected),state.selected.id+'.json');});
  $('export-filtered').addEventListener('click',()=>exportData({run_id:DATA.meta.run_id,interpretation:DATA.meta.interpretation,units:'metres',quaternion_order:'XYZW',filters:{kind:$('kind-filter').value,rack:$('rack-filter').value,outcome:$('outcome-filter').value,search:$('search').value},count:state.filtered.length,records:state.filtered.map(exportRecord)},'dishwasher-filtered-poses.json'));
  document.addEventListener('keydown',e=>{if(/INPUT|SELECT|TEXTAREA|BUTTON/.test(e.target.tagName))return;if(e.key==='ArrowRight'){e.preventDefault();stopPlayback();step(1);}if(e.key==='ArrowLeft'){e.preventDefault();stopPlayback();step(-1);}if(e.code==='Space'&&e.target===$('viewport')){e.preventDefault();play();}});
  document.addEventListener('visibilitychange',()=>{if(document.hidden)stopPlayback();});
  window.viewerDebug=Object.freeze({get selectedId(){return state.selected?.id||null;},get requestedStage(){return state.stage;},get displayedStage(){return state.selected?effective(state.selected):null;},get visibleIds(){return state.filtered.map(t=>t.id);},get webgl(){return !!renderer;},get playing(){return state.playing;},get cameraPosition(){return camera?.position.toArray();},get renderedObjectCount(){return mainView?.objects.children.length||0;},get displayedPose(){const stage=state.selected&&effective(state.selected);return stage?structuredClone(state.selected[stage].dish):null;}});
  initScene();filter();
})();
