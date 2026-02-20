const $ = (s,e=document)=>e.querySelector(s);
const $$ = (s,e=document)=>Array.from(e.querySelectorAll(s));

const storageKey="fusion_learning_progress_bw_v1";
const loadProgress=()=>{try{return JSON.parse(localStorage.getItem(storageKey)||"{}")}catch(e){return {}}};
const saveProgress=(p)=>localStorage.setItem(storageKey,JSON.stringify(p));
const setChecked=(id,val)=>{const p=loadProgress();p[id]=!!val;saveProgress(p);};
const isChecked=(id)=>!!loadProgress()[id];

const escapeHtml=(s)=>s.replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;");

const LINKS={
  barfootBook:"https://www.cambridge.org/core/books/state-estimation-for-robotics/00E53274A2F1E6CC1A55CA5C3D1C9718",
  barfootPdf:"https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser24.pdf",
  solaESKF:"https://arxiv.org/abs/1711.02508",
  vinsMono:"https://arxiv.org/abs/1708.03852",
  fastlio:"https://github.com/hku-mars/FAST_LIO",
  fastlioPdf:"https://raw.githubusercontent.com/hku-mars/FAST_LIO/main/doc/Fast_LIO_2.pdf",
  liosam:"https://github.com/TixiaoShan/LIO-SAM",
  underact:"https://underactuated.csail.mit.edu/index.html",
  underactOCW:"https://ocw.mit.edu/courses/6-832-underactuated-robotics-spring-2022/",
  modernRob:"https://modernrobotics.northwestern.edu/nu-gm-book-resource/",
  isaacDocs:"https://docs.isaacsim.omniverse.nvidia.com/",
  mujocoDocs:"https://mujoco.readthedocs.io/",
  drakeDocs:"https://drake.mit.edu/doxygen_cxx/",
  autowareDoc:"https://autowarefoundation.github.io/autoware-documentation/main/home/",
  autowareUni:"https://autowarefoundation.github.io/autoware_universe/main/",
  inekf2019:"https://arxiv.org/abs/1904.09251",
};

const ALL_TASK_IDS={};
const addTaskIds=(items)=>items.forEach(it=>ALL_TASK_IDS[it.id]=true);
const computeCompletion=(prefix)=>{
  const ids=Object.keys(ALL_TASK_IDS).filter(k=>k.startsWith(prefix));
  if(!ids.length) return {done:0,total:0,rate:0};
  const done=ids.filter(isChecked).length;
  const total=ids.length;
  return {done,total,rate:Math.round(done/total*100)};
};

function renderChecklist(items){
  addTaskIds(items);
  const lis=items.map(it=>{
    const checked=isChecked(it.id)?"checked":"";
    return `<li><label class="checkline"><input type="checkbox" data-check="${it.id}" ${checked}/><span>${it.text}</span></label></li>`;
  }).join("");
  return `<ul class="checklist">${lis}</ul>`;
}

function modulePage({title,prefix,goals,learn,formulas,code,papers,tasks,pitfalls,next}){
  return { title, render(){
    const prog=computeCompletion(prefix);
    const paperHtml=papers.map(p=>`<li><a href="${p.url}" target="_blank" rel="noreferrer">${escapeHtml(p.name)}</a> <span class="muted">— ${escapeHtml(p.note||"")}</span></li>`).join("");
    const learnHtml=learn.map(x=>`<li>${x}</li>`).join("");
    const goalHtml=goals.map(x=>`<li>${x}</li>`).join("");
    const pitHtml=(pitfalls||[]).map(x=>`<li>${x}</li>`).join("");
    return `
      <div class="card">
        <h3>${escapeHtml(title)}</h3>
        <p class="muted">模块完成度：${prog.done}/${prog.total}（${prog.rate}%）</p>
        <div class="progress"><div class="bar"><div style="width:${prog.rate}%"></div></div><span>勾选任务会保存到本地浏览器</span></div>
      </div>

      <div class="grid">
        <div class="card"><h3>学习目标</h3><ul class="checklist">${goalHtml}</ul></div>
        <div class="card"><h3>知识点清单</h3><ul class="checklist">${learnHtml}</ul></div>
      </div>

      <details open><summary>关键公式</summary><div class="hr"></div>${formulas}</details>
      <details open><summary>代码骨架（最小可跑）</summary><div class="hr"></div>${code}</details>
      <details><summary>论文/教材/官方文档</summary><div class="hr"></div><ul class="checklist">${paperHtml}</ul></details>
      <details open><summary>本模块必须完成的任务（可打勾）</summary><div class="hr"></div>${renderChecklist(tasks)}</details>
      <details><summary>工程坑点</summary><div class="hr"></div><ul class="checklist">${pitHtml||"<li>暂无</li>"}</ul></details>
      ${next?`<div class="card"><h3>下一步</h3><p>${next}</p></div>`:""}
    `;
  }};
}

const PAGES={};
PAGES.home={title:"总览与使用说明", render(){
  return `
    <div class="card">
      <h3>融合学习站（黑白简约版）</h3>
      <p>目标：<b>自动驾驶 + 具身智能 融合工程师</b>（估计 + 控制 + 系统）。</p>
      <div class="callout">学习闭环：读论文/教材 → 推导关键公式 → 写最小可跑代码 → 仿真/数据验证 → 复盘总结。</div>
      <p class="muted">建议按左侧顺序：Year1 → Year2 → Year3。勾选任务会自动保存。</p>
    </div>
    <div class="grid">
      <div class="card">
        <h3>三年主线</h3>
        <ul class="checklist">
          <li><b>状态估计</b>：IMU → ESKF → LIO/VIO → InEKF/因子图</li>
          <li><b>控制</b>：PID/LQR → MPC/优化 → WBC</li>
          <li><b>系统能力</b>：同步/延迟/资源/排障 → 可部署、可复现</li>
        </ul>
      </div>
      <div class="card">
        <h3>快速开始</h3>
        <ol class="checklist">
          <li>打开 Year1-M1，做完 3 个任务</li>
          <li>把代码放进你的仓库 /code 目录</li>
          <li>周末写一页“实验报告”（曲线 + 结论）</li>
        </ol>
      </div>
    </div>
  `;
}};

PAGES.resources={title:"教材/论文/工具索引", render(){
  return `
    <div class="card">
      <h3>权威教材</h3>
      <ul class="checklist">
        <li><a href="${LINKS.barfootBook}" target="_blank" rel="noreferrer">State Estimation for Robotics（Barfoot）</a> / <a href="${LINKS.barfootPdf}" target="_blank" rel="noreferrer">PDF</a></li>
        <li><a href="${LINKS.modernRob}" target="_blank" rel="noreferrer">Modern Robotics</a></li>
        <li><a href="${LINKS.underact}" target="_blank" rel="noreferrer">Underactuated Robotics</a> / <a href="${LINKS.underactOCW}" target="_blank" rel="noreferrer">MIT OCW</a></li>
      </ul>
    </div>
    <div class="grid">
      <div class="card">
        <h3>估计（必读）</h3>
        <ul class="checklist">
          <li><a href="${LINKS.solaESKF}" target="_blank" rel="noreferrer">Solà — ESKF</a></li>
          <li><a href="${LINKS.vinsMono}" target="_blank" rel="noreferrer">VINS-Mono</a></li>
          <li><a href="${LINKS.fastlio}" target="_blank" rel="noreferrer">FAST-LIO</a> / <a href="${LINKS.fastlioPdf}" target="_blank" rel="noreferrer">FAST-LIO2 PDF</a></li>
          <li><a href="${LINKS.liosam}" target="_blank" rel="noreferrer">LIO-SAM</a></li>
          <li><a href="${LINKS.inekf2019}" target="_blank" rel="noreferrer">Contact-aided InEKF</a></li>
        </ul>
      </div>
      <div class="card">
        <h3>仿真/系统</h3>
        <ul class="checklist">
          <li><a href="${LINKS.isaacDocs}" target="_blank" rel="noreferrer">Isaac Sim 文档</a></li>
          <li><a href="${LINKS.mujocoDocs}" target="_blank" rel="noreferrer">MuJoCo 文档</a></li>
          <li><a href="${LINKS.drakeDocs}" target="_blank" rel="noreferrer">Drake 文档</a></li>
          <li><a href="${LINKS.autowareDoc}" target="_blank" rel="noreferrer">Autoware 文档</a> / <a href="${LINKS.autowareUni}" target="_blank" rel="noreferrer">Universe</a></li>
        </ul>
      </div>
    </div>
  `;
}};

PAGES.y1_m1=modulePage({
  title:"Year1-M1：旋转与 IMU 模型",
  prefix:"y1_m1_",
  goals:["理解 IMU 的物理测量含义（gyro/acc）","掌握四元数姿态更新（必须能写代码）","能解释：静止时加速度计为什么≈9.81"],
  learn:["坐标系 world/body 与右手系约定","R / quaternion / ω^ / Exp(·) 的关系","IMU测量模型：bias + noise + specific force","姿态离散更新（小角度近似）"],
  formulas:`<div class="callout">\\(\\omega_m = \\omega + b_g + n_g\\)<br/>\\(a_m = R^\\top (a - g) + b_a + n_a\\)</div>
\\[\\omega^\\wedge =
\\begin{bmatrix}0 & -\\omega_z & \\omega_y\\\\\\omega_z & 0 & -\\omega_x\\\\-\\omega_y & \\omega_x & 0\\end{bmatrix}\\]
\\[R_{k+1} = R_k\\exp(\\omega^\\wedge\\Delta t),\\quad
q_{k+1} = q_k \\otimes \\delta q,\\; \\delta q\\approx\\begin{bmatrix}1\\\\\\tfrac12\\omega\\Delta t\\end{bmatrix}\\]`,
  code:`<pre><code>#include &lt;Eigen/Dense&gt;
#include &lt;Eigen/Geometry&gt;

Eigen::Quaterniond UpdateQuat(const Eigen::Quaterniond&amp; q,
                              const Eigen::Vector3d&amp; omega, double dt){
  Eigen::Vector3d dtheta = omega * dt;
  Eigen::Quaterniond dq(1.0, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
  return (q * dq).normalized();
}</code></pre>`,
  papers:[
    {name:"Barfoot — State Estimation for Robotics", url:LINKS.barfootBook, note:"估计体系与旋转建模"},
    {name:"Solà — ESKF（arXiv）", url:LINKS.solaESKF, note:"旋转误差与ESKF必读"},
  ],
  tasks:[
    {id:"y1_m1_t1", text:"恒定 ωz 积分 10s：yaw 是否 ≈ ωz·t（写误差曲线）"},
    {id:"y1_m1_t2", text:"加入 gyro bias：观察 yaw 漂移速度，并估计漂移率"},
    {id:"y1_m1_t3", text:"静止 IMU：用加速度估计重力方向做初始化，并写解释"},
  ],
  pitfalls:["忘记四元数归一化导致数值漂移","重力方向/坐标系定义混乱（z-up vs z-down）","把加速度计当“世界加速度”而忽略比力"],
  next:"进入 Year1-M2：完整 IMU 传播（p,v,q,b）。"
});

PAGES.y1_m2=modulePage({
  title:"Year1-M2：IMU 传播（p,v,q,b）",
  prefix:"y1_m2_",
  goals:["实现名义状态传播：p,v,q,ba,bg","理解 bias/噪声导致积分漂移","为 ESKF 做好传播模块"],
  learn:["去 bias：ω=ωm-bg, ab=am-ba","aw=R*ab+g","离散积分：v,p 更新","时间戳与 dt 可靠性"],
  formulas:`\\[\\omega = \\omega_m - b_g,\\quad a_b = a_m - b_a,\\quad a_w = R a_b + g\\]
\\[v_{k+1} = v_k + a_w\\Delta t\\]
\\[p_{k+1} = p_k + v_k\\Delta t + \\tfrac12 a_w\\Delta t^2\\]`,
  code:`<pre><code>struct Nominal {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); // world_R_body
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
};

void Propagate(Nominal&amp; X, const Eigen::Vector3d&amp; acc_m,
               const Eigen::Vector3d&amp; gyro_m, double dt){
  Eigen::Vector3d omega = gyro_m - X.bg;
  Eigen::Vector3d acc_b = acc_m  - X.ba;

  X.q = UpdateQuat(X.q, omega, dt);
  Eigen::Matrix3d R = X.q.toRotationMatrix();
  Eigen::Vector3d g(0,0,-9.81);

  Eigen::Vector3d acc_w = R * acc_b + g;
  X.p = X.p + X.v*dt + 0.5*acc_w*dt*dt;
  X.v = X.v + acc_w*dt;
}</code></pre>`,
  papers:[
    {name:"Barfoot PDF（IMU传播章节）", url:LINKS.barfootPdf, note:"配合代码实现"},
    {name:"Solà — ESKF（实现细节）", url:LINKS.solaESKF, note:"误差定义与注入"},
  ],
  tasks:[
    {id:"y1_m2_t1", text:"只做传播：跑一段IMU数据，画 p/v/yaw 漂移曲线"},
    {id:"y1_m2_t2", text:"实现简单“静止检测”，用均值估计 bias（粗略即可）"},
    {id:"y1_m2_t3", text:"做一次故障注入：dt 错/单位错/重力错，并记录定位方法"},
  ],
  pitfalls:["dt 错（单位/时间戳不单调）导致发散","重力方向未初始化导致速度爆炸"],
  next:"进入 Year1-M3：ESKF（误差状态KF）推导+框架。"
});

PAGES.y1_m3=modulePage({
  title:"Year1-M3：ESKF（误差状态KF）",
  prefix:"y1_m3_",
  goals:["区分名义状态与误差状态","会写预测与更新（K、P）","能接入简单观测（外部位置）"],
  learn:["误差状态：δp,δv,δθ,δba,δbg","旋转误差注入：R<-R*Exp(δθ^)","Joseph形式协方差更新","观测：残差 r=z-h(x)"],
  formulas:`\\[P = FPF^\\top + Q\\]
\\[K = PH^\\top (HPH^\\top + R)^{-1}\\]
\\[\\delta x = K\\,r,\\quad P \\leftarrow (I-KH)P(I-KH)^\\top + KRK^\\top\\]`,
  code:`<pre><code>struct ESKF {
  Nominal X;
  Eigen::MatrixXd P;

  void predict(const Eigen::Vector3d&amp; acc_m, const Eigen::Vector3d&amp; gyro_m, double dt){
    Propagate(X, acc_m, gyro_m, dt);
    // TODO: build F, Q (or F,G,Q)
    // P = F * P * F.transpose() + Q;
  }

  void updatePosition(const Eigen::Vector3d&amp; p_meas, const Eigen::Matrix3d&amp; R_meas){
    // r = p_meas - X.p
    // H = [I 0 0 0 0]
    // dx = K r ; inject ; P update (Joseph)
  }
};</code></pre>`,
  papers:[
    {name:"Solà — ESKF（必读）", url:LINKS.solaESKF, note:"最清晰的ESKF讲解"},
    {name:"Barfoot — KF/ESKF/批量估计", url:LINKS.barfootBook, note:"建立整体估计观"},
  ],
  tasks:[
    {id:"y1_m3_t1", text:"实现最小 ESKF：外部观测为位置 p_meas（跑通+画 P 对角线）"},
    {id:"y1_m3_t2", text:"验证 Q 变大时：P 增长更快、滤波更信任观测"},
    {id:"y1_m3_t3", text:"写一页：ESKF 如何纠正积分误差（用你自己的话）"},
  ],
  pitfalls:["旋转误差注入错误（不要直接加 quaternion）","P 非对称/非正定导致崩溃"],
  next:"你可以继续让我把 Year1 后续（LIO/VIO/FAST-LIO/Autoware）和 Year2/Year3 全部填满。"
});

function placeholder(title,text){
  return {title, render:()=>`<div class="card"><h3>${escapeHtml(title)}</h3><p>${text}</p><p class="muted">（占位页：你随时可以让我把这一模块的公式/代码/论文/任务补全。）</p></div>`};
}
PAGES.y2_m1=placeholder("Year2-M7：运动学（FK/IK/J）","覆盖：齐次变换、FK、数值IK、Jacobian、奇异性与仿真实操。");
PAGES.y2_m2=placeholder("Year2-M8：控制入门（PID/LQR）","覆盖：PID调参方法、离散LQR、对比实验与指标体系。");
PAGES.y2_m3=placeholder("Year2-M9：MPC/优化","覆盖：QP形式、滚动优化、约束处理与实时性。");
PAGES.y2_m4=placeholder("Year2-M10：刚体动力学","覆盖：M(q),C(q,qdot),g(q)、接触力与仿真验证。");
PAGES.y2_m5=placeholder("Year2-M11：仿真平台","覆盖：Isaac/MuJoCo/Drake 的最小闭环与数据记录。");
PAGES.y2_m6=placeholder("Year2-M12：腿式估计（InEKF）","覆盖：接触观测、不可观测性、InEKF核心思想与实验。");
PAGES.y3_m1=placeholder("Year3-M13：系统工程","覆盖：同步/延迟/profiling/复现实验流程与架构图。");
PAGES.y3_m2=placeholder("Year3-M14：高级估计","覆盖：InEKF/因子图/鲁棒核/工程取舍。");
PAGES.y3_m3=placeholder("Year3-M15：WBC","覆盖：接触力QP、摩擦锥、任务优先级与仿真验证。");
PAGES.y3_m4=placeholder("Year3-M16：学习控制（选修）","覆盖：RL/IL入门、domain randomization、评估与安全。");
PAGES.y3_m5=placeholder("Year3-M17：作品集/面试","覆盖：项目故事线、实验报告模板、面试题库与答案。");

PAGES.tracker={title:"进度与打卡", render(){
  const sections=[{name:"Year1（估计基础）",prefix:"y1_"},{name:"Year2（控制/动力学/仿真）",prefix:"y2_"},{name:"Year3（系统/高级/作品集）",prefix:"y3_"}];
  const cards=sections.map(s=>{
    const c=computeCompletion(s.prefix);
    return `<div class="card"><h3>${s.name}</h3><p class="muted">完成度：${c.done}/${c.total}（${c.rate}%）</p>
      <div class="progress"><div class="bar"><div style="width:${c.rate}%"></div></div><span>${c.rate}%</span></div></div>`;
  }).join("");
  return `<div class="grid">${cards}</div>
    <div class="card"><h3>今日打卡（10分钟）</h3>
      <ul class="checklist"><li>今天做了什么（读/推/写/跑/测）？</li><li>遇到的最大问题是什么？证据是什么？</li><li>明天最小下一步是什么（可执行）？</li></ul>
    </div>`;
}};

let currentPage="home";
const setActiveNav=(page)=>$$(".navItem").forEach(b=>b.classList.toggle("active",b.dataset.page===page));

function bindCheckboxes(){
  $$("input[type='checkbox'][data-check]").forEach(cb=>{
    cb.addEventListener("change", ()=>{
      setChecked(cb.dataset.check, cb.checked);
      renderPage(currentPage, true);
    }, {once:true});
  });
}

function renderPage(page, keepScroll=false){
  const y=window.scrollY;
  const p=PAGES[page]||PAGES.home;
  currentPage=page;
  $("#pageTitle").textContent=p.title;
  $("#root").innerHTML=p.render();
  setActiveNav(page);
  bindCheckboxes();
  if(window.MathJax && window.MathJax.typesetPromise) window.MathJax.typesetPromise();
  if(keepScroll) window.scrollTo(0,y);
}

$$(".navItem").forEach(btn=>btn.addEventListener("click", ()=>renderPage(btn.dataset.page)));

const searchInput=$("#q");
const doSearch=()=>{
  const term=searchInput.value.trim().toLowerCase();
  $$(".navItem").forEach(b=>{
    const t=b.textContent.toLowerCase();
    b.style.display=(!term || t.includes(term)) ? "" : "none";
  });
};
searchInput.addEventListener("input", doSearch);

document.addEventListener("keydown",(e)=>{
  if((e.ctrlKey||e.metaKey) && e.key.toLowerCase()==="k"){ e.preventDefault(); searchInput.focus(); }
});

$("#expandAll").addEventListener("click", ()=>$$("details").forEach(d=>d.open=true));
$("#collapseAll").addEventListener("click", ()=>$$("details").forEach(d=>d.open=false));
$("#resetProgress").addEventListener("click", ()=>{
  if(!confirm("确定清空所有勾选进度？")) return;
  localStorage.removeItem(storageKey);
  renderPage(currentPage);
});

renderPage("home");
