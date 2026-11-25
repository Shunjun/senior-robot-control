# 共享障碍层与 Nav2 未知区域导航设计

2026-11-25。**用户已明确“可以就按这个实施吧”，以下行为及主技术选择已批准。** 本文归档正式设计；产品尚未实施。执行采用 Orca 分阶段开发/审核，负责人亲阅本设计与[实施计划](../plans/2026-11-25-shared-obstacle-layers.md)后派发开发。本次仅文档归档，不启动实施。

本文取代[旧前沿导航设计](2026-11-25-frontier-navigation-design.md)及其计划；旧文保留历史主体，不能继承其中的前沿选择、临时等待、语义否决或未知必须先变 free 的要求。

## 目标与已确认行为

使用现有 Nav2 导航到地图内的未知区域，持续建图、避障和重新规划。取消自研前沿选择、临时障碍几秒或30秒等待与自定义通道封锁。Nav2 的原有失败与任务生命周期沿现有接口处理，不把某次局部规划失败直接解释成目标永久不可达。

不识别人、箱子、小车等物体类别。使用经车辆自身运动补偿后的多帧几何变化，判断轮廓当前稳定还是存在自身运动。三次有效雷达命中只证明量测累积，不单独证明静止。

- 稳定物体可进入持久地图，包括孤立的箱子等；不要求与墙相连。
- 后来观测到它移动，撤销原位置的稳定贡献，当前实际位置继续显示和避障。
- 曾经移动的物体重新稳定后，可以经过完整稳定判断再次入图。
- 固定结构超出扫描范围仍保留并参与规划。重访获得真实空闲证据时，可以移除原占据。
- 临时障碍只在当前有效扫描覆盖中显示和参与规划。离开覆盖、被遮挡或观测失效后，撤销其当前贡献；撤销不等于证明该处空闲。

## 实现方式与取舍

推荐：同一 mapping 生命周期与写入 owner、一次轻量感知计算，产生固定底图和当前扫描两个输出层。它们共用定位、坐标转换及扫描资格，不另建第二套SLAM。

考虑过直接清洗 Karto 的全部历史扫描和图约束，但现有接口不支持可靠的局部撤销，成本和定位影响尚未验证；因此不默认采用。也考虑过每帧发送固定图与当前障碍合成的整张栅格，但地图扩大后复制与传输开销随整图增长，因此推荐小型 Nav2 当前层插件。

以上是负责人推荐且随本设计获批的技术选择；物体稳定/移动的产品行为来自用户要求。定位内部原始历史可以保留供定位使用，但不能用于重新生成已撤销的产品占据。若实测历史动态点影响定位，再按证据处理局部定位问题，不预先扩大为全部图清洗。

## 数据流与职责

合格雷达扫描先产生最新当前快照，再做有界几何判断。当前障碍不等待多帧分类，第一帧合格命中即可参与避障。

1. 扫描入口沿用原来源、时钟、精确采样TF、外参、自体过滤与新鲜度资格。保存地图、没有新关键帧或分类稍慢，都不能阻塞当前扫描更新。
2. 几何模块采用角顺序分簇、可信odom补偿、有限候选的局部轮廓关联。遮挡、分裂合并、关联歧义或补偿不足时暂不升级固定，不阻止当前有效命中进入避障。
3. mapping owner唯一负责固定层的 occupied/free/unknown 更新、稳定支持、运动撤销和持久化。每次固定修订不再强制要求新增一个SLAM关键帧。
4. ROS发布桥只安装owner确认的固定版本，保持唯一合格 `/map` 发布者；当前快照通过独立最新槽位传播，不排在保存RPC锁后。
5. UI读取同源固定底图与当前扫描，区别显示稳定占据和当前障碍。不改变已批准页面结构；UI抽样不得成为导航输入。

当前扫描身份与有效期独立于固定地图版本。扫描更新不触发整张底图重新拉取；旧来源、旧参考或迟到结果不能刷新当前有效期。回环校正重投影固定证据，不能被误判为物体运动。

## 固定层的更新与保存

固定图保存真实的三态，而不是把所有未占据区域认作可通行。有限有效命中之前的可见射线可形成空闲证据；遮挡之后、缺束和断流不能形成空闲证据。无回波能否证明某段可见空闲取决于实际雷达来源协议，不能仅凭数值猜测；具备明确合格无回波语义后才在其有效距离内融合。

同一格可能有独立的墙体和物体支持。运动撤销只删除被推翻的支持；仍有可信墙体支持时继续occupied，不能因free冲突或普通数值max被降成unknown。撤销后无占据支持、又无当前有效free证据的格为unknown。

free区域按局部图块保存，支持摘要与短期关联有容量限制。重复观测不无限新增永久轨迹或支持记录。达到容量限制时停止新增固定升级并报告，不能悄悄删除墙体，也不能停止当前避障输入。

保存原子绑定地图身份、参考代次、graph/anchor水位、固定修订与已应用证据水位。当前扫描不进入长期占据checkpoint。恢复、导出、重栅格化和选图均读取同一权威固定结果，不能fallback到Karto历史重新恢复已移除的障碍。新格式提高版本，旧加载器应明确拒绝。

旧地图保留，不删除原档或批量撤销导航资格。工程迁移建议：若旧格式缺少保存的free但原导航依赖native三态，仅在首次旧格式迁移时冻结一次有界旧三态结果，标为legacy来源并应用已有撤销记录；此后成为可更新的权威副本。更新后的版本不再从raw重画。旧占据不冒充新算法已经判稳，后续真实观测逐步确认、撤销或更新。原档历史查看与当前副本身份分开。

## Nav2 接入

global与local costmap均按固定底图、当前扫描层、膨胀层组合。移除原 `/scan` 障碍层保留旧格的重复贡献路径。薄当前层插件不识别、不跟踪、不规划，只保留当前有效扫描和上次实际渲染范围。

当前障碍移动、失效或离开覆盖后，下一次costmap组合撤去旧贡献与旧膨胀，再铺固定底图；不能用写free的方式擦除固定墙。local滚动窗口同样使用固定底图。真实传感器有效距离与global范围明确配置，不把原2.5米marking范围冒称全部雷达范围。

启用规划器allow_unknown，同时迁移导航目标准入和地图身份传递；允许free/unknown目标，occupied及旧身份仍拒绝。栅格范围之外不能直接当成已有unknown格：实施计划要核实原有地图扩展能力和规划范围，明确可表达的目标边界，不能仅翻allow_unknown就宣称任意图外目标可导航。航点创建沿用原有free约束，本轮不默改任务编辑行为。

unknown允许通过不等于允许穿越已知障碍。必须在实际部署Nav2版本验证unknown与lethal混合足迹，不能仅凭配置或mock数组通过宣布安全。Nav2包版本与真实参数当前尚未取得。

## 资源约束与验证

使用线性扫描分簇、有界局部候选和有限帧历史，不引入语义模型、全局最优匹配、完整ICP、永久tracker或每帧全历史重绘。固定层只更新dirty图块；回环重投影、保存与恢复的峰值单独测量。

实施前计划须细化现有每scan入口、同anchor更新、持久格式及发布桥接口。契约确定后，几何纯算法、Nav2薄插件和最新UI消费者可隔离并行；固定层更新与保存恢复由同一责任人顺序收口。统一速度alpha/k与反向ramp属于另一独立任务，本设计不增加来源分支。

验收需要实际穿过生产owner/adapter的静止无关键帧与存盘场景、稳定→移动→撤销→保存重载闭环、真实旧格式迁移，以及部署Nav2的旧格/膨胀撤销和未知通道对照。四核并发SLAM、控制、MPPI、UI和存盘测量延迟、CPU、RSS、锁等待及队列，不编造性能承诺。真实雷达误差、raw历史对定位影响及Nav2混合足迹仍属待验证项，不能用单元测试替代。

## 归档状态及源码细化

以下细化吸收几何草稿独立审核发现的问题，不改变上述获批行为。原临时审计材料的必要结论在本节与[计划的证据、契约及验收表](../plans/2026-11-25-shared-obstacle-layers.md)归档；实施不依赖临时文件链接。

- 核验基线：功能分支 `feat/frontier-navigation`，文档HEAD `390bd6235c4e0dab434ce93f69dd11691e6a0dff`，产品基线 `11a26906c9831b000b09e2279ad7ef4ea514d903`。2026-11-25只读远端查询确认 main=`66125807b7189ef28b250168df944a308eac9d33`，PR42已MERGED、非Draft，UI头=`1adcaf8503c3c78a4267a2e2f6f5b74cddb291a1`。本轮未fetch/merge/checkout；开发前重查并正常merge最新main进原功能分支，保留UI最终结构。
- [`bounded_scan_ingress.cpp`](../../../ros2/src/senior_mapping/src/slam/bounded_scan_ingress.cpp)资格入口与[`slam_graph_node.cpp`](../../../ros2/src/senior_mapping/src/slam/slam_graph_node.cpp)的graph gate目前串联。每scan事实须在原source/clock/exact TF/static mount及cached geometry验证后、graph gate前产出；不能只从`ReadGraphCommit`取Q。adapter最新槽与Q发布订阅不经过`RosMappingBackend.request`通用锁或`NativeSession._save`锁。
- 原native入口和capture当前共用默认执行器，移到gate前仍可能被序列化阻塞；计划明确独立合格入口callback group/受生命周期管理的单线程，与graph owner间用有界队列及immutable资格/几何见证，分类不占入口锁。
- [`store.py`](../../../ros2/src/senior_mapping/senior_mapping/store.py)现`_prepare_ingest`既要求连续graph sequence，也要求anchor集合每次恰增1。应保留graph连续性，另加同一已验证anchor cut下的证据水位/fixed修订，不伪造keyframe。source gap使短期稳定窗口重启，不给漏帧累计稳定证据。
- [`occupancy.py`](../../../ros2/src/senior_mapping/senior_mapping/occupancy.py)、[`evidence.py`](../../../ros2/src/senior_mapping/senior_mapping/evidence.py)与[`references.py`](../../../ros2/src/senior_mapping/senior_mapping/references.py)现只存occupied、单格记录。需要同owner内的独立支持ID、真实free图块、运动撤销原因；被撤支持原anchor与触发free观测anchor分开。free按区域保守重投影，边界可退unknown，仍可信occupied支持优先。
- 旧free不能在新occupied覆盖后仍当有效清空证明；支持写入须废止该局部旧free代次。运动撤销无新的真实free时回unknown。重复稳定观测刷新有界支持摘要；短期track身份不成为永久全局物体身份。
- [`native_session.py`](../../../ros2/src/senior_mapping/senior_mapping/native_session.py)改返回值不足以改变另进程的`/map`。[`navigation_bridge.py`](../../../ros2/src/senior_mapping/senior_mapping/navigation_bridge.py)须显式安装经owner确认的fixed cut，并同时复核MapRef/R/boot/graph/anchor/fixed/evidence水位；生产固定输出禁止native raster fallback。去掉“必须存在free格”作为图资格的代用判断，保留原定位/reference和唯一发布者校验。
- 内层occupancy、外层bundle及checkpoint均升级版本；实际V1没有保存完整native free，不能假定旧PGM自带free。首次V1迁移可冻结一次原有有界native三态，标`legacy-derived`并叠加removals；有独立可信旧三态时保留其legacy来源。之后绝对层损坏则拒该cut，不能raw重建、不能默退旧cut伪装最新。当前副本与原档历史选择分开，保持原档和原导航资格规则。
- Nav2插件保留的是“上次实际渲染域”，不能用上条收到的Q代替；合并旧域及本周期固定快照域，master重置后重铺fixed→Q→inflation。expiry、reset/disable、rolling origin变化均保留一次清旧域机会；当前hit覆盖unknown为lethal，不作`max(255,254)`。local StaticLayer使用`use_maximum=false`，并对部署实现验证。
- [`simulation_motion.launch.py`](../../../ros2/src/senior_sim/launch/simulation_motion.launch.py)确有map_server/AMCL静态图装配；这不是已复现双发布。保留该模式，进入可更新mapping时由原生命周期交接发布资格，不能并列启动第二地图写owner。
- 预算区分：共享Workspace读取上限来自[`workspace_limits.py`](../../../vehicle_core/workspace_limits.py)，是16,777,216格；现native导航raster/bridge上限1,048,576格，不能混称。scan≤2048束、pending=8；store总records≤65536、单批changes≤8192；旧occupancy tracks≤100000不能保证free或支持总容量。native raster≤16384 scans是读预算，不能声称graph写入已受同一上限限制。
- 真实构建证据另有差异：mapping CMake要求`slam_toolbox 2.6.10 EXACT`，package.xml却写`version_eq=2.6.8`；应在部署取证后按原2.6.10源码契约修正声明并做安装态校验。Nav2包未精确锁版，本机没有`/opt/ros`和`ros2`，本轮不能真实构建/安装或验Nav2。

- 实际IPC帧上限8192字节，不能把2048点硬塞原state.get。计划将UI现96点界面明确为有hit_count/preview_count的同源当前扫描预览，沿角桶最近hit选取；Nav2仍接收完整Q。原UI结构沿用，独立期限通过原state/协议/cloud缓存传递，缓存驻留及重复快照不得续期；跨主机单程网络延迟边界须如实记录，不冒称与同机Nav2截止完全同步。

## 验收与边界

[实施计划](../plans/2026-11-25-shared-obstacle-layers.md)列出四部分、原owner、具体文件/接口、有界状态、S01–S16和逐任务文件并集预测。先取得部署Nav2/补丁版本、传感器无回波协议及odom/雷达误差证据，再冻结依赖它们的参数和ABI。代码能锁定的身份/三态规则可先实现；没有物理证据不能靠虚构阈值宣布可靠判稳，没有部署环境不能拿mock或本地跳过代替Nav2验证。

地图外目标尚无经查证的目标驱动扩图接口：本轮只承诺已发布有限栅格内unknown。自然建图扩展后，新落入栅格的目标可重新按当前G准入；任意图外坐标需要负责人依据范围/owner证据另行收敛。速度alpha/k及来源无关反向ramp单列独立依赖，不作为共享层实施前置条件。raw对定位的影响、四核资源及实车行为继续明确为未验。
