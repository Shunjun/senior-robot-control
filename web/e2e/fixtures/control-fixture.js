/* Safe protocol fixture, loaded only by browser tests/static preview, never imported by production. */
(() => {
  if(window.__wheelFixture) return;
  const calls=[];
  const ref={map_id:'fixture-map',reference_revision:2};
  const waypoints=['A','B','C'].map((waypoint_id,i)=>({waypoint_id,kind:'waypoint',status:'active',x:i*.4,y:.5,yaw:0}));
  const driving={low_linear_mps:.5,high_linear_mps:1,low_angular_rps:1,high_angular_rps:1,degraded_linear_mps:.3,reverse_linear_mps:.3};
  const logging={level:'INFO',retention_seconds:604800,debug_enabled:false,debug_restore_seconds:300};
  const settings={snapshot:{driving,logging,driving_version:2,logging_version:7,debug_original_level:null,debug_deadline_utc:null},can_apply_driving:true,capabilities:{driving_defaults:{...driving},driving_minimum:Object.fromEntries(Object.keys(driving).map(k=>[k,0])),driving_maximum:Object.fromEntries(Object.keys(driving).map(k=>[k,2])),logging_defaults:{...logging},retention_min_seconds:3600,retention_max_seconds:2592000,debug_restore_min_seconds:60,debug_restore_max_seconds:3600}};
  const tasks=[{task_id:'fixture-route',name:'定时巡检',revision:3,enabled:true}];
  const definitions=new Map([['fixture-route',{task_id:'fixture-route',name:'定时巡检',waypoint_ids:['A','B','C'],schedule:{kind:'cron',expression:'15 9 * * 1,3'},timezone:'Asia/Shanghai',enabled:true,revision:3,created_at:'2026-11-24T00:00:00Z'}]]);
  sessionStorage.setItem('senior-control.csrf','fixture-csrf');
  window.fetch=async(input,init)=>{
    const url=String(input);calls.push({http:url,method:init?.method??'GET'});
    if(url==='/api/session' && init?.method==='DELETE')return new Response(null,{status:204});
    if(url==='/api/robots')return new Response(JSON.stringify({robots:[{id:'fixture-A',name:'安全测试车辆 A',online:true},{id:'fixture-B',name:'安全测试车辆 B',online:false}]}));
    if(url==='/api/pairing-codes')return new Response(JSON.stringify({code:'ABCD234567',expires_in:37}),{status:201});
    if(url.startsWith('/api/'))return new Response('{}',{status:404});
    throw new Error('Safety fixture blocked fetch: '+url);
  };
  class FixtureSocket {
    static OPEN=1;readyState=0;onopen=null;onclose=null;onmessage=null;onerror=null;timer=null;sequence=0;
    constructor(url){
      this.url=String(url);this.robotId=new URL(this.url).searchParams.get('robot_id');
      setTimeout(()=>{if(this.readyState===3)return;this.readyState=1;this.onopen?.({});
        if(!this.url.includes('/logs')){this.emit({v:1,type:'control.connection',robot_id:this.robotId,connection_epoch:1});this.snapshot();this.timer=setInterval(()=>this.snapshot(),1000);}
      },0);
    }
    emit(frame){this.onmessage?.({data:JSON.stringify(frame)});}
    snapshot(){
      this.sequence++;const now=Date.now()/1000;
      this.emit({v:1,robot_id:this.robotId,session_id:'fixture',connection_epoch:1,request_id:'snapshot',seq:this.sequence,expires_at:now+10,type:'snapshot',payload:{connected:true,localized:true,mapping_protocol_version:2,map_ref:ref,map_generation:'reference-7',map_grid_id:'raster-23',home:null,home_revision:0,task_revision:3,tasks,runnable_task_ids:[],execution_status:'idle',collected_at:now,last_synced_at:now,stale:false,navigation_ready:true,robot_pose:{x:0,y:0,yaw:.6},scan_points:[{x:.8,y:.6},{x:.82,y:.5}],vehicle_ui:{schema:1,sequence:this.sequence,mode:'idle',reason:'standby',estop_revision:1,fault_revision:0,confirmed_estop_status:'clear',body_reference:{kind:'independent_body',average:{vx:.5,vy:0,wz:.1},source:'fixture_body',age_seconds:0,stale:false},wheel_linear_mps:.5,task:null,manual_authority:null,allowed_actions:['acquire'],capabilities:['runtime.v1','motion.intent.v1','settings.v1','logs.v1','task.names.v1']},performance:{sequence:this.sequence,sampled_at:now,interval_s:1,scope:'procfs_visible',cpu_count:4,cpu_percent:25,memory_used_bytes:536870912,memory_total_bytes:1073741824,modules:[{name:'video',status:'inactive',cpu_percent:null,rss_bytes:null,process_count:0}]}}});
    }
    send(raw){
      const q=JSON.parse(raw);calls.push(q);
      if(this.url.includes('/logs')){
        const entry={sequence:1,kind:'event',level:'WARNING',module:'mapping',summary:'安全 fixture：逐行日志\n<img src=x onerror=alert(1)>',detail:'保留原文',fields_json:'{"source":"fixture"}',source_utc:1000,received_utc:1001,request_id:'fixture-request',task_id:'fixture-route',loss_reason:null,loss_count:0};
        const data=q.action==='download'?{data:JSON.stringify(entry)+'\n',next_cursor:1,has_more:false,gaps:[]}:{entries:[entry],next_cursor:1,high_water:1,has_more:false,scanned:1,gaps:[]};
        setTimeout(()=>this.emit({ok:true,data}),0);return;
      }
      if(q.type.startsWith('video.') || q.type==='drive.intent')return;
      let data,ok=true,error=null;const identity={robot_id:this.robotId,map_ref:ref,grid_id:'raster-23'};
      if(q.type==='map.catalog') {
        const entry=(map_ref,name)=>({map_ref,name,checkpoint_id:'a'.repeat(32),home_id:'fixture-home',version:1,saved_at:1000});
        data={robot_id:this.robotId,entries:[entry(ref,'安全测试地图'),entry({map_id:'fixture-saved',reference_revision:1},'已保存测试地图')],selection:{map_ref:ref,generation:7,state:'ready'},runtime_available:true,phase:'mapping'};
      }
      else if(q.type==='map.manifest') data={...identity,width:96,height:80,resolution:.05,origin:{x:-2,y:-2,yaw:.23},tile_size:32,navigation_ready:true,waypoint_writes:true};
      else if(q.type==='map.tile'){
        const {tile_x, tile_y}=q.payload,width=32,height=Math.min(32,80-tile_y*32);
        const cells=Array.from({length:width*height},(_,i)=>{const x=tile_x*32+i%32,y=tile_y*32+Math.floor(i/32);return x<5||y<5||x>90||y>74?-1:x===8||x===86||y===9||y===70?100:0;});
        data={...identity,tile_x,tile_y,width,height,cells};
      } else if(q.type==='waypoint.list')data={...identity,waypoints,next_cursor:null};
      else if(q.type==='task.get')data={...identity,...definitions.get(q.payload.task_id)};
      else if(q.type==='settings.get')data=settings;
      else if(q.type==='settings.apply'){
        const {group,values,request_id}=q.payload;settings.snapshot[group]={...values};settings.snapshot[group+'_version']++;
        data={request_id,group,reason:'applied',snapshot:settings.snapshot,capabilities:settings.capabilities};
      }else if(q.type==='control.acquire')data={authority_id:'fixture-authority',state:'prepared'};
      else if(q.type==='waypoint.create')waypoints.push({...q.payload.body,kind:'waypoint',status:'active'});
      else if(q.type==='task.create' || q.type==='task.update'){
        const body=q.payload.body;
        const previous=definitions.get(body.task_id);
        const revision=previous?previous.revision+1:1;
        definitions.set(body.task_id,{...body,revision});
        const summary={task_id:body.task_id,name:body.name,revision,enabled:body.enabled};
        const index=tasks.findIndex(task=>task.task_id===body.task_id);
        if(index<0)tasks.push(summary);else tasks[index]=summary;
      }
      else if(q.type==='task.enable'){const task=tasks.find(task=>task.task_id===q.payload.body.task_id);if(task){task.enabled=q.payload.body.enabled;task.revision++;}}
      else if(['drive.stop','control.release','task.delete','safety.estop'].includes(q.type)){}
      else {ok=false;error='unavailable';}
      setTimeout(()=>this.emit({...q,type:'response',payload:{request_type:q.type,ok,error,status:ok?'succeeded':'failed',...(data?{data}:{})}}),0);
    }
    close(){if(this.timer)clearInterval(this.timer);this.readyState=3;}
  }
  window.WebSocket=FixtureSocket;
  window.__wheelFixture={calls,settings,tasks,waypoints,definitions};
  document.addEventListener('DOMContentLoaded',()=>{const badge=document.createElement('div');badge.textContent='安全协议 fixture · 无真实连接/运动';badge.className='text-xs font-sans';badge.style='position:fixed;bottom:0;left:0;z-index:100;background:#674323;color:white;padding:4px 8px;pointer-events:none';document.body.appendChild(badge);});
})();
