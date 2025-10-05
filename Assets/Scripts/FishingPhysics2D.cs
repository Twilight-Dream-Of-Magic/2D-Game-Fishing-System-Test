// =========================================================
// FishingPhysics2D.cs — Unity 6 (6000.x)  重构版（完整替换）
// 关键修复：
//  - 竿梢速度在 FixedUpdate 采样并注入（不再出现 vTip=0）
//  - Flight 严格“松绳上限” + 早落水（绷紧若干帧 / 离屏）
//  - 可选：随时左键“强制开始收线”（reelAnytime）
//  - 可选：抛出后短暂保持竿“前倾”稳定（holdRodForward）
//  - Launch Feel：tipBoost / extraBoost / powerCurve 可调
// Inspector 字段名保持与你现场一致
// =========================================================

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;

[DisallowMultipleComponent]
[RequireComponent(typeof(LineRenderer))]
public class FishingPhysics2D : MonoBehaviour
{
    [Header("Events（外部可挂回调）")]
    public FishingEvents events = new FishingEvents();

    [Header("Debug & HUD（开发期可视化）")]
    [Tooltip("是否绘制屏幕左上角状态信息")] public bool showDebugOverlay = false;
    [Tooltip("是否绘制速度/距离曲线")] public bool showGraphs = false;
    [Tooltip("是否打印阶段切换日志")] public bool logPhaseChanges = true;
    [Tooltip("曲线采样容量（帧）")] [Range(60, 1024)] public int historyCapacity = 300;
    [Tooltip("调试绘制比例（像素/单位）")] [Range(20f, 200f)] public float graphScale = 80f;
    [Tooltip("开发快捷键：F1 HUD, F2 Graph, F3 reelAnytime, F4 holdRodForward")] public bool devHotkeys = true;

    [Header("Configuration（打包参数）")]
    [Tooltip("Awake 时是否应用下方 config 的参数覆盖")] public bool applyConfigOnAwake = true;
    public FishingConfig config = new FishingConfig();

	// ---------- Inspector（名称保持原样，兼容现有场景） ----------
	[Header("UI & Layers")]
	public Slider chargeSlider;
	public LayerMask waterMask;

    [Header("Rod Chain (refs)")]
    public Transform RodRoot; // 根节点（旋转枢轴）
    public Rigidbody2D seg1, seg2, seg3, rodTip;
	public HingeJoint2D hinge1, hinge2, hinge3;

	[Header("Bobber & Rope (refs)")]
	public Rigidbody2D bobber;            // 浮标（CircleCollider2D）
	public DistanceJoint2D rope;          // 挂在 bobber 上，connectedBody=rodTip

    [Header("Cast Tuning")]
    [Min(0.01f)] public float minCast = 1.8f;
    [Min(0.01f)] public float maxCast = 7.2f;
    [Tooltip("每秒蓄力进度 0..1")] public float chargeSpeed = 1.6f;
    [Tooltip("用于估计额外前向速度的时间窗")] public float flightTime = 0.35f;
    [Tooltip("最小飞行时间（早于此不落水判定）")] public float landTimeout = 0.45f;

    [Header("Drag (Unity 6 uses linearDamping)")]
    public float airDrag = 0.38f;
    public float waterDrag = 4.5f;

    [Header("Reel Tuning")]
    [Tooltip("distance 每秒缩短量")] public float reelRate = 6.5f;
    [Tooltip("沿绳方向额外拉力")] public float reelForce = 14f;

    [Header("Whip（甩竿）")]
    public float whipMotorSpeed = 1100f;
    public float whipPulse = 0.14f;
    public bool releaseAtForwardApex = true;
    [Header("Whip Animation（根节点动画）")]
    [Tooltip("Whip 阶段是否同时动画 RodRoot 角度")] public bool animateRodRootInWhip = true;
    [Tooltip("Whip 阶段的目标前倾角（度）")] public float forwardLeanDeg = 18f;

    [Header("Flight Gravity")]
    [Tooltip("飞行期的重力系数（顶视角 0.10~0.30）")] public float flightGravity = 0.18f;

    [Header("Launch Feel（手感增强）")]
    [Tooltip("竿梢速度增益")] public float tipBoost = 1.30f;
    [Tooltip("解析额外速度增益")] public float extraBoost = 1.00f;
    [Tooltip("蓄力曲线 >1 变硬，<1 变软")] public float powerCurve = 1.20f;

    [Header("Launch Feel（竿身旋转）")]
    [Tooltip("蓄满时的最大后仰角（度，负值为向后）")] public float maxBackAngleDeg = 55f;
    [Tooltip("Charging 时竿身旋转的平滑时间（秒）")] public float rotationSmoothTime = 0.06f;

    [Header("Flight Lock（视觉友好落水）")]
    [Tooltip("飞行允许的额外松弛比例")][Range(0f, 0.5f)] public float slackRatio = 0.06f;
    [Tooltip("连续多少帧“已绷紧”就直接落水")][Range(1, 8)] public int tautFramesToLand = 2;
    [Tooltip("离屏外溢容差（0~0.2）")][Range(0f, 0.2f)] public float offscreenMargin = 0.05f;
    [Tooltip("只在命中水面时判定落水（演示需求：抛进水里就结束）")] public bool landOnlyOnWater = true;
    [Header("Water Landing（混合逻辑）")]
    [Tooltip("命中水后需要停留的最短时间，避免“刚碰水就落”")] public float waterSettleDelay = 0.2f;
    [Tooltip("水中锁长的速度阈值，低于则可落")] public float waterLockSpeed = 0.8f;

    [Header("Line Renderer")]
    [Min(2)] public int lineSegments = 20;
	[Min(16)] public int pixelsPerUnit = 128;
	public int sortingOrder = 10;
	public Color lineColor = new Color(1f, 0.1f, 0.6f, 1f);
	public bool drawDebugLine = true;

    [Header("Home Pose（复位外观）")]
    public bool snapRodOnReelFinish = true;
    public bool snapRodAlsoFromEmergencyReset = true;
    [Tooltip("落水时仅复位鱼竿（不回收浮标），用于演示抛进水里即结束")] public bool snapRodOnLand = true;

    [Header("Interaction（交互开关）")]
    [Tooltip("飞行期左键是否可随时强制收线（不等超时/绷紧）")] public bool reelAnytime = true;

    [Header("Rod Forward Hold（抛出后前倾保持）")]
    [Tooltip("抛出后短时间把根节角度限制在前倾区间")] public bool holdRodForward = true;
    [Tooltip("保持时长（秒）")] public float holdForwardDuration = 0.55f;
    [Tooltip("相对释放瞬间角度，允许后仰的角度上限（度）")] [Range(0f, 90f)] public float holdRodBackLimit = 40f;
    [Tooltip("相对释放瞬间角度，允许前倾的角度上限（度）")] [Range(0f, 90f)] public float holdRodForwardLimit = 25f;

    [Header("Rod Limits（根节角度限制）")]
    [Tooltip("是否启用根节的角度限制，防止 360° 旋转")] public bool enforceRodLimits = true;
    [Tooltip("根节最小角（度，负为向后）")] public float rodMinAngleDeg = -55f;
    [Tooltip("根节最大角（度，正为向前）")] public float rodMaxAngleDeg = 28f;

	// ------------------------- 内部状态 -------------------------
	enum Phase { Idle, Charging, Whip, Flight, Landed, Reeling }
    Phase phase = Phase.Idle;
    PhaseMachine fsm = new PhaseMachine();

	LineRenderer lr;
	float charge01, castTimer, plannedLen;
	float uiFadeT; bool uiFading;
	float whipT, tipFwdPrev;             // 甩竿计时 & 前向度量
	int tautFrames;                      // 连续“已绷紧”计数

	// 竿梢“真实”速度（由 Seg3 刚体在 FixedUpdate 计算）
	Vector2 tipVelFixed;
    // 鼠标决定的当前抛投方向（相对竿梢）和左右符号
    Vector2 castDirCached = Vector2.right;
    float sideSignCached = 1f; // 右:+1，左:-1

    // Charging 阶段围绕 RodRoot 的后仰控制（避免 360° 抖动）
    float currentBackAngleDeg;
    float rotSmoothVelDeg;
    Quaternion rodRootHomeRot;

	// Home Pose 快照
	struct TPose
	{
		public Vector3 pos; public Quaternion rot; public Vector3 scale;
		public TPose(Transform t) { pos = t.localPosition; rot = t.localRotation; scale = t.localScale; }
		public void Apply(Transform t) { t.localPosition = pos; t.localRotation = rot; t.localScale = scale; }
	}
	TPose seg1Home, seg2Home, seg3Home, tipHome, bobberHome;
	bool homeCaptured;

    // Hinge 恢复快照（用于 holdRodForward 临时限制）
    bool hingeHoldActive;
    bool hingePrevUseLimits, hingePrevUseMotor;
    JointAngleLimits2D hingePrevLimits;
    JointMotor2D hingePrevMotor;

    // Debug 运行统计与历史
    int castCount;          // 总抛次数
    float maxCastDistance;  // 历史最远落水长度
    float lastCastLen;      // 上一次落水长度
    float lastReleaseSpeed; // 上一次释放初速
    // 水命中计时
    bool sawWater;          // 本次飞行是否已首次命中水
    float waterHitT;        // 首次命中水的相对时间（castTimer）

    float[] speedHistory;   // 每帧 tip/bobber 速度采样
    float[] distHistory;    // 每帧 tip-bobber 距离采样
    int histIndex;
    GUIStyle hudStyle;

	// ------------------------- 生命周期 -------------------------
    void Awake()
	{
		lr = GetComponent<LineRenderer>();
		lr.useWorldSpace = true;
		if (!lr.material) lr.material = new Material(Shader.Find("Sprites/Default"));
        // 先应用配置，再设置渲染器属性，避免被覆盖
        if (applyConfigOnAwake && config != null) ApplyConfig();
        lr.startColor = lr.endColor = lineColor;
        lr.sortingOrder = sortingOrder;
		ConfigureLine();
        fsm.Init(Phase.Idle);

		if (rope)
		{
			rope.connectedBody = rodTip;
			rope.autoConfigureDistance = false;
			rope.maxDistanceOnly = true; // 只限制“最大长度”
			rope.enabled = false;
		}

		if (bobber)
		{
			bobber.gravityScale = 0f;
			bobber.linearDamping = airDrag;
			bobber.angularDamping = 0.05f;
			bobber.bodyType = RigidbodyType2D.Kinematic;
			bobber.linearVelocity = Vector2.zero;   // Unity 6
		}

		if (chargeSlider)
		{
			chargeSlider.minValue = 0f; chargeSlider.maxValue = 1f;
			chargeSlider.wholeNumbers = false;
			chargeSlider.value = 0f; chargeSlider.gameObject.SetActive(false);
		}

		CaptureHome();
		ResetAll(true);
	}

	void OnValidate()
	{
		maxCast = Mathf.Max(maxCast, minCast + 0.01f);
		lineSegments = Mathf.Clamp(lineSegments, 2, 128);
		pixelsPerUnit = Mathf.Max(16, pixelsPerUnit);
		if (lr) ConfigureLine();
	}

	// 物理步里**直接**用刚体求竿梢速度（旋转→切向速度）
	void FixedUpdate()
	{
		if (seg3 && rodTip)
		{
			// 关键：用动态段 RigidBody2D 的 GetPointVelocity 取世界上一点的线速度
			// 即便 RodTip 本身是 Kinematic 也能得到真实速度
			tipVelFixed = seg3.GetPointVelocity(rodTip.position);
		}
	}

	// --------------------------- 主循环 ---------------------------
    void Update()
    {
        // 右键紧急复位（先彻底取消所有延迟/协程）
        if (Input.GetMouseButtonDown(1)) { CancelInvoke(); StopAllCoroutines(); ResetAll(snapRodAlsoFromEmergencyReset); return; }

        // 统一更新一次基于鼠标的抛投方向/左右符号（供全流程使用）
        UpdateCastDirCached();

        // 把阶段逻辑分发给 fsm（内部仍调用当前类的方法）
        fsm.Tick(this, Time.deltaTime);

        // 对 RodRoot 的角度进行全局硬限制（无论哪个阶段）
        if (enforceRodLimits) EnforceRodRootLimits();

        // Pre-Release：刚性整体
        EnsureAttachedDuringPreRelease();

        TickUiFade(Time.deltaTime);
        UpdateLineRenderer();
        Watchdog();
    }

	// --------------------------- 状态实现 ---------------------------
    void BeginCharging()
	{
        ChangePhase(Phase.Charging);
		charge01 = 0f; castTimer = 0f;

		if (chargeSlider) { chargeSlider.value = 0; chargeSlider.gameObject.SetActive(true); }

		AttachToTipKeepWorld();
		bobber.transform.localPosition = Vector3.zero;
		if (rope) rope.enabled = false;

        if (events?.onChargeStart != null) events.onChargeStart.Invoke();
        Debug.Log("[Fishing] Charging start", this);
	}

    void TickCharging(float dt)
	{
        charge01 = Mathf.Min(1f, charge01 + Mathf.Max(0f, chargeSpeed) * dt);
        if (chargeSlider) chargeSlider.value = charge01;

        // 禁用根节马达，Charging 仅用确定性后仰角呈现
        if (hinge1) hinge1.useMotor = false;

        // 依据鼠标方向确定“前/后”的符号（右为前→后仰为负；左为前→后仰为正）
        // 依据统一缓存的鼠标方向与左右
        float sideSign = sideSignCached;

        // 目标后仰角（按蓄力幅度）
        float t = Mathf.Pow(Mathf.Clamp01(charge01), Mathf.Max(0.0001f, powerCurve));
        float targetDeg = -Mathf.Abs(maxBackAngleDeg) * t * sideSign;
        if (enforceRodLimits)
        {
            targetDeg = Mathf.Clamp(targetDeg, rodMinAngleDeg, rodMaxAngleDeg);
        }
        currentBackAngleDeg = Mathf.SmoothDampAngle(currentBackAngleDeg, targetDeg, ref rotSmoothVelDeg, Mathf.Max(0.0001f, rotationSmoothTime));
        ApplySeg1BackAngle(currentBackAngleDeg);
	}

    void BeginWhip()
	{
        ChangePhase(Phase.Whip);
		whipT = 0f; tipFwdPrev = -999f;
		plannedLen = minCast + (maxCast - minCast) * Mathf.Pow(Mathf.Clamp01(charge01), Mathf.Max(0.05f, powerCurve));
		if (chargeSlider) StartUiFadeOut();
		PulseHingeTowardMouse();
        if (events?.onWhipBegin != null) events.onWhipBegin.Invoke();
        Debug.Log($"[Fishing] WHIP charge={charge01:F2} lenPlan={plannedLen:F2}", this);
	}

	void TickWhip(float dt)
	{
		whipT += dt;
		bobber.transform.localPosition = Vector3.zero;

        float sideSign = sideSignCached;

		// 用 FixedUpdate 缓存的真实竿梢速度
		Vector2 vTip = tipVelFixed;
        float tipFwd = (vTip.sqrMagnitude < 1e-6f) ? -999f : Vector2.Dot(vTip.normalized, castDirCached);
		bool apex = releaseAtForwardApex && tipFwdPrev > -998f && tipFwdPrev > tipFwd; // 前向速度开始回落
		tipFwdPrev = tipFwd;

        // 根节点前倾动画：将 backAngle 朝前倾过渡（与鼠标方向一致）
        if (animateRodRootInWhip && RodRoot)
        {
            float p = Mathf.Clamp01(whipT / Mathf.Max(0.02f, whipPulse));
            float targetDeg = Mathf.Lerp(currentBackAngleDeg, Mathf.Abs(forwardLeanDeg) * sideSign, p);
            if (enforceRodLimits) targetDeg = Mathf.Clamp(targetDeg, rodMinAngleDeg, rodMaxAngleDeg);
            currentBackAngleDeg = Mathf.SmoothDampAngle(currentBackAngleDeg, targetDeg, ref rotSmoothVelDeg, Mathf.Max(0.0001f, rotationSmoothTime));
            ApplySeg1BackAngle(currentBackAngleDeg);
        }

        if (apex || whipT >= whipPulse) ReleaseNow(castDirCached, vTip);
	}

    void ReleaseNow(Vector2 castDir, Vector2 vTip)
	{
		// 解析推算的额外速度（与规划线长 / 阻尼 / 时间窗有关）
		float v0 = EstimateInitialSpeed(plannedLen, bobber ? bobber.linearDamping : airDrag, flightTime);

		DetachFromTipKeepWorld();
		bobber.linearDamping = airDrag;
		bobber.gravityScale = flightGravity; // 抛物线感觉

		// 初速度：竿梢切向 + 解析前向
		Vector2 vInit = vTip * Mathf.Max(0f, tipBoost) + castDir * v0 * Mathf.Max(0f, extraBoost);
		bobber.linearVelocity = vInit;

		// 飞行：严格“最大长度上限”，略加松弛
		if (rope)
		{
			rope.enabled = true;
			rope.maxDistanceOnly = true;
			rope.distance = Mathf.Min(plannedLen * (1f + Mathf.Abs(slackRatio)), maxCast);
		}

        ChangePhase(Phase.Flight);
        castTimer = 0f; tautFrames = 0; sawWater = false; waterHitT = 0f;

        lastReleaseSpeed = vInit.magnitude;
        if (events?.onRelease != null) events.onRelease.Invoke();
        Debug.Log($"[Fishing] RELEASE vTip={vTip.magnitude:F2} v0={v0:F2}", this);

        // 抛出后短暂保持前倾（可选）
        if (holdRodForward) HoldRodForwardTemporarily();
	}

	void TickFlight(float dt)
	{
		castTimer += dt;
		if (!bobber || !rope || !rodTip) return;

        // 视口外溢：如果仅水面落水关闭，则强制落水；否则放行由水面判定
        if (!landOnlyOnWater && IsOffscreen(bobber.position, offscreenMargin)) { ForceLand(); return; }

        // 混合逻辑：记录首次命中水的时刻，但不立即落水
        bool inWaterNow = Physics2D.OverlapPoint(bobber.position, waterMask) != null;
        if (inWaterNow && !sawWater) { sawWater = true; waterHitT = castTimer; }

		float tipTo = Vector2.Distance(rodTip.position, bobber.position);
		bool tight = tipTo >= rope.distance - 0.01f;
		tautFrames = tight ? (tautFrames + 1) : 0;

        // 如果不是“只在水面落水”，可用拉紧若干帧的早落水
        if (!landOnlyOnWater && tautFrames >= Mathf.Max(1, tautFramesToLand) && castTimer >= landTimeout * 0.4f) { ForceLand(); return; }

        // 落水条件（混合）：
        // - landOnlyOnWater=true：需要命中水，且满足（到时 or 入水后延迟达到，且[已绷紧 或 速度慢 或 回向竿梢]）
        // - landOnlyOnWater=false：原有条件即可
        Vector2 toTip = (Vector2)rodTip.position - bobber.position;
        bool inward = Vector2.Dot(toTip, bobber.linearVelocity) > 0f || bobber.linearVelocity.sqrMagnitude < 0.0004f;
        bool slowInWater = bobber.linearVelocity.sqrMagnitude <= waterLockSpeed * waterLockSpeed;
        bool waterDelayOk = sawWater && (castTimer - waterHitT) >= Mathf.Max(0f, waterSettleDelay);

        if (landOnlyOnWater)
        {
            if (inWaterNow && (castTimer >= landTimeout || (waterDelayOk && (tight || inward || slowInWater))))
            { ForceLand(); return; }
        }
        else
        {
            if (castTimer >= landTimeout)
            {
                if (inWaterNow || (tight && inward)) { ForceLand(); return; }
            }
        }
	}

	bool IsOffscreen(Vector3 worldPos, float margin)
	{
		if (!Camera.main) return false;
		var v = Camera.main.WorldToViewportPoint(worldPos);
		return v.x < -margin || v.x > 1f + margin || v.y < -margin || v.y > 1f + margin;
	}

    void ForceLand()
	{
		float cur = Vector2.Distance(rodTip.position, bobber.position);
        if (rope) { rope.enabled = true; rope.maxDistanceOnly = false; rope.distance = cur; }
		bobber.linearVelocity = Vector2.zero;
		bobber.angularVelocity = 0f;
		bobber.linearDamping = waterDrag;
		bobber.gravityScale = 0f;
        ChangePhase(Phase.Landed);
        castCount++;
        lastCastLen = cur;
        maxCastDistance = Mathf.Max(maxCastDistance, cur);
        if (events?.onLand != null) events.onLand.Invoke();
        if (snapRodOnLand) RestoreRodPoseOnly();
        Debug.Log($"[Fishing] LANDED lockLen={cur:F2}", this);
	}

	void TryBeginReelFromFlight()
	{
        if (!Input.GetMouseButtonDown(0) || phase != Phase.Flight || rope == null || rodTip == null) return;

        if (reelAnytime)
        {
            // 直接强制落水并开始收线
            ForceLand();
            BeginReel();
            Debug.Log("[Fishing] Begin reel from Flight (anytime)", this);
            return;
        }

        // 原有逻辑：超时且已绷紧
        if (castTimer < landTimeout) return;
        float tipTo = Vector2.Distance(rodTip.position, bobber.position);
        if (tipTo < rope.distance - 0.02f) return; // 未绷紧
        ForceLand();
        BeginReel();
        Debug.Log("[Fishing] Begin reel from Flight (tight & timeout)", this);
	}

    void BeginReel()
	{
        ChangePhase(Phase.Reeling);
		if (rope) rope.maxDistanceOnly = false; // 绝对长度模式：distance 变短会“拖回来”
        if (events?.onReelStart != null) events.onReelStart.Invoke();
        Debug.Log("[Fishing] Reel start", this);
	}

	void TickReeling(float dt)
	{
		if (rope) rope.distance = Mathf.Max(0f, rope.distance - Mathf.Max(0f, reelRate) * dt);

		if (rodTip && bobber)
		{
			Vector2 toTip = (Vector2)rodTip.position - bobber.position;
			if (toTip.sqrMagnitude > 1e-6f) bobber.AddForce(toTip.normalized * reelForce, ForceMode2D.Force);

			if (toTip.magnitude <= 0.12f || (rope && rope.distance <= 0.02f))
			{
				if (rope) { rope.enabled = false; rope.distance = 0f; }
				AttachToTipKeepWorld();
				bobber.transform.localPosition = Vector3.zero;
				bobber.gravityScale = 0f;

                if (snapRodOnReelFinish) RestoreHomePoseInstant();

                ChangePhase(Phase.Idle);
                if (events?.onReelFinish != null) events.onReelFinish.Invoke();
                Debug.Log("[Fishing] Reel finish", this);
			}
		}
	}

	// --------------------------- 复位 / Home Pose ---------------------------
	void CaptureHome()
	{
		if (homeCaptured || seg1 == null || seg2 == null || seg3 == null || rodTip == null || bobber == null) return;
        seg1Home = new TPose(seg1.transform);
		seg2Home = new TPose(seg2.transform);
		seg3Home = new TPose(seg3.transform);
		tipHome = new TPose(rodTip.transform);
		bobberHome = new TPose(bobber.transform);
		homeCaptured = true;
        if (RodRoot) rodRootHomeRot = RodRoot.localRotation;
	}

	void RestoreHomePoseInstant()
	{
		if (!homeCaptured) return;
		var b1 = seg1.bodyType; var b2 = seg2.bodyType; var b3 = seg3.bodyType;
		seg1.bodyType = seg2.bodyType = seg3.bodyType = RigidbodyType2D.Kinematic;
		seg1Home.Apply(seg1.transform);
		seg2Home.Apply(seg2.transform);
		seg3Home.Apply(seg3.transform);
		tipHome.Apply(rodTip.transform);
		seg1.bodyType = b1; seg2.bodyType = b2; seg3.bodyType = b3;
        if (RodRoot) RodRoot.localRotation = rodRootHomeRot;
		AttachToTipKeepWorld();
		bobber.transform.localPosition = Vector3.zero;
	}

// 仅复位鱼竿，不处理浮标（用于落水即结束的展示效果）
void RestoreRodPoseOnly()
{
    if (!homeCaptured) return;
    var b1 = seg1.bodyType; var b2 = seg2.bodyType; var b3 = seg3.bodyType;
    seg1.bodyType = seg2.bodyType = seg3.bodyType = RigidbodyType2D.Kinematic;
    seg1Home.Apply(seg1.transform);
    seg2Home.Apply(seg2.transform);
    seg3Home.Apply(seg3.transform);
    tipHome.Apply(rodTip.transform);
    seg1.bodyType = b1; seg2.bodyType = b2; seg3.bodyType = b3;
}

    void ResetAll(bool snapRod)
	{
        // —— 强制进入 Idle，并把 FSM 重新初始化，切断上一阶段的 Tick —— //
        ChangePhase(Phase.Idle);
        fsm.Init(Phase.Idle);

        // —— 取消任何延迟/Invoke/协程（StopHingeMotor、ReleaseHoldRodForward 等）—— //
        CancelInvoke();
        StopAllCoroutines();
        hingeHoldActive = false; // 防止误判

        // —— 关闭所有关节的 motor/limits —— //
        ResetHinge(hinge1); ResetHinge(hinge2); ResetHinge(hinge3);

        // —— 基本标量清零 —— //
        charge01 = 0f; castTimer = 0f; plannedLen = 0f;
        uiFading = false; uiFadeT = 0f; whipT = 0f; tipFwdPrev = -999f; tautFrames = 0;
        tipVelFixed = Vector2.zero;

        // —— 线关节：彻底关闭并清零 —— //
        if (rope) { rope.enabled = false; rope.maxDistanceOnly = true; rope.distance = 0f; }

        // —— 清零所有刚体的线/角速度（不仅仅是 bobber）—— //
        ZeroRB(seg1); ZeroRB(seg2); ZeroRB(seg3); ZeroRB(rodTip); ZeroRB(bobber);

        // —— 复位外观（先把段设为 Kinematic，套用快照，再还原）—— //
        if (snapRod) RestoreHomePoseInstant();
        else {
            // 即便不 snapRod，也要把 bobber 正确回挂
            AttachToTipKeepWorld();
            bobber.transform.localPosition = Vector3.zero;
        }

        // —— Bobber 回到“整体状态” —— //
        if (bobber)
        {
            bobber.bodyType = RigidbodyType2D.Kinematic;
            bobber.linearVelocity = Vector2.zero;
            bobber.angularVelocity = 0f;
            bobber.linearDamping = airDrag;
            bobber.gravityScale = 0f;
            AttachToTipKeepWorld();
            bobber.transform.localPosition = Vector3.zero;
        }

        // —— UI & 线渲染器 —— //
        if (chargeSlider) { chargeSlider.value = 0f; chargeSlider.gameObject.SetActive(false); }
        if (lr)
        {
            lr.enabled = false;
            lr.positionCount = Mathf.Max(2, lineSegments) + 1;
            Vector3 p = rodTip ? (Vector3)rodTip.position : transform.position;
            for (int i = 0; i < lr.positionCount; i++) lr.SetPosition(i, p);
        }

        if (events?.onReset != null) events.onReset.Invoke();
        Debug.Log("[Fishing] ResetToIdle", this);
	}

	// --------------------------- 附着/解绑 ---------------------------
	void AttachToTipKeepWorld()
	{
		var t = bobber.transform;
		t.SetParent(rodTip.transform, true); // 保持世界位姿
		t.position = rodTip.position;        // 再贴紧
		t.rotation = rodTip.transform.rotation;
		t.localScale = bobberHome.scale;     // 防“越抛越大/小”
		bobber.bodyType = RigidbodyType2D.Kinematic;
		bobber.linearVelocity = Vector2.zero;
		bobber.angularVelocity = 0f;
		bobber.linearDamping = airDrag;
	}

	void DetachFromTipKeepWorld()
	{
		var t = bobber.transform;
		t.SetParent(null, true);             // 释放瞬间解绑
		bobber.bodyType = RigidbodyType2D.Dynamic;
	}

	void EnsureAttachedDuringPreRelease()
	{
		if (phase == Phase.Idle || phase == Phase.Charging || phase == Phase.Whip)
		{
			if (bobber.transform.parent != rodTip.transform) AttachToTipKeepWorld();
			bobber.transform.localPosition = Vector3.zero;
		}
}
    // 统一更新鼠标方向缓存（相对竿梢）
    void UpdateCastDirCached()
    {
        Vector2 origin = rodTip ? (Vector2)rodTip.position : (Vector2)transform.position;
        Vector2 mouse = Camera.main ? (Vector2)Camera.main.ScreenToWorldPoint(Input.mousePosition) : origin + Vector2.right;
        castDirCached = (mouse - origin).sqrMagnitude > 1e-6f ? (mouse - origin).normalized : Vector2.right;
        sideSignCached = Mathf.Sign(Mathf.Abs(castDirCached.x) < 1e-4f ? 1f : castDirCached.x);
    }
    // 围绕 RodRoot 旋转（不再直接旋转 seg1）
    void ApplySeg1BackAngle(float backDeg)
    {
        if (!RodRoot) return;
        // 硬限制角度范围
        if (enforceRodLimits)
            backDeg = Mathf.Clamp(backDeg, rodMinAngleDeg, rodMaxAngleDeg);
        // 基于 RodRoot 的 home 旋转
        RodRoot.localRotation = rodRootHomeRot * Quaternion.Euler(0f, 0f, backDeg);
    }
    // --------------------------- 竿前倾保持 ---------------------------
    void HoldRodForwardTemporarily()
    {
        if (!hinge1 || hingeHoldActive) return;

        // 记录当前设置
        hingePrevUseLimits = hinge1.useLimits;
        hingePrevUseMotor = hinge1.useMotor;
        hingePrevLimits = hinge1.limits;
        hingePrevMotor = hinge1.motor;

        // 以释放瞬间角度为中心，设置窄窗口的角度限制
        float baseAngle = Mathf.Clamp(hinge1.jointAngle, rodMinAngleDeg, rodMaxAngleDeg); // 当前角度（度）
        var limits = new JointAngleLimits2D
        {
            min = enforceRodLimits ? rodMinAngleDeg : baseAngle - Mathf.Abs(holdRodBackLimit),
            max = enforceRodLimits ? rodMaxAngleDeg : baseAngle + Mathf.Abs(holdRodForwardLimit)
        };

        hinge1.useMotor = false; // 停止进一步驱动，避免与限制冲突
        hinge1.limits = limits;  // 必须先设置 limits 再启用 useLimits
        hinge1.useLimits = true;

        hingeHoldActive = true;
        Invoke(nameof(ReleaseHoldRodForward), Mathf.Clamp(holdForwardDuration, 0.05f, 2.0f));
    }

    void ReleaseHoldRodForward()
    {
        if (!hinge1 || !hingeHoldActive) return;
        // 恢复原始设置
        hinge1.useLimits = hingePrevUseLimits;
        hinge1.limits = hingePrevLimits;
        hinge1.useMotor = hingePrevUseMotor;
        hinge1.motor = hingePrevMotor;
        hingeHoldActive = false;
        // 保险：确保完全关闭
        ResetHinge(hinge1);
    }


	// --------------------------- 实用小块 ---------------------------
	void PulseHingeTowardMouse()
	{
		if (!hinge1 || !seg1 || !rodTip) return;
		Vector2 origin = rodTip.position;
		Vector2 mouse = Camera.main ? (Vector2)Camera.main.ScreenToWorldPoint(Input.mousePosition) : origin + Vector2.right;
		Vector2 castDir = (mouse - origin).sqrMagnitude > 1e-6f ? (mouse - origin).normalized : Vector2.right;

		Vector2 axis = ((Vector2)rodTip.position - (Vector2)seg1.position).normalized;
		float sign = Mathf.Sign(Vector2.SignedAngle(axis, castDir));
		var m = hinge1.motor;
		m.motorSpeed = sign * Mathf.Abs(whipMotorSpeed);
		m.maxMotorTorque = 1e6f;
		hinge1.motor = m;          // 注意：必须回写给 joint 才生效
		hinge1.useMotor = true;
		Invoke(nameof(StopHingeMotor), Mathf.Clamp(whipPulse * 1.1f, 0.02f, 0.2f));
	}
	void StopHingeMotor() { if (hinge1) hinge1.useMotor = false; }

	// v(t)=v0 e^{-k t}，s=(v0/k)(1-e^{-kT}) → v0 = L k / (1 - e^{-kT})
	static float EstimateInitialSpeed(float length, float linearDamping, float T)
	{
		float L = Mathf.Max(0f, length);
		float k = Mathf.Max(0f, linearDamping);
		float t = Mathf.Max(1e-3f, T);
		if (k < 1e-3f) return L / t;
		float denom = 1f - Mathf.Exp(-k * t);
		if (denom < 1e-4f) return L * k;
		return L * k / denom;
	}

	// --------------------------- 视觉线 ---------------------------
	void ConfigureLine()
	{
		float w = 1f / Mathf.Max(1, pixelsPerUnit);
		lr.widthMultiplier = 1f;
		lr.widthCurve = AnimationCurve.Constant(0f, 1f, w);
		lr.positionCount = Mathf.Max(2, lineSegments) + 1;
	}

	void UpdateLineRenderer()
	{
		if (!lr || !rodTip || !bobber) return;
		int N = Mathf.Max(2, lineSegments);
		if (lr.positionCount != N + 1) lr.positionCount = N + 1;

		Vector3 A = rodTip.position, B = bobber.position;
		float L = Vector2.Distance(A, B);
		float Lc = Mathf.Min(L, maxCast * 1.5f);
		bool sagging = (phase == Phase.Landed || phase == Phase.Reeling);
		float sag = sagging ? 0.10f : 0f;

		for (int i = 0; i <= N; i++)
		{
			float t = i / (float)N;
			Vector3 p = Vector3.Lerp(A, B, t);
			p.y -= sag * Lc * Mathf.Sin(Mathf.PI * t);
			lr.SetPosition(i, p);
		}
		if (drawDebugLine) Debug.DrawLine(A, B, Color.magenta, 0f, false);
		lr.enabled = !(phase == Phase.Idle || phase == Phase.Charging);
	}

	// --------------------------- Safety ---------------------------
	void Watchdog()
	{
		if (phase == Phase.Idle || !rodTip || !bobber) return;
		float dist = Vector2.Distance(rodTip.position, bobber.position);
		if (dist > maxCast * 3f || float.IsNaN(dist))
		{ // 极限保护
			Debug.LogWarning($"[Fishing] Watchdog: dist={dist:F2} too large, RESET.", this);
            CancelInvoke(); StopAllCoroutines(); ResetAll(true);
		}
	}

	// --------------------------- UI 渐隐 ---------------------------
	void StartUiFadeOut() { uiFading = chargeSlider != null; uiFadeT = 0f; }
	void TickUiFade(float dt)
	{
		if (!uiFading || !chargeSlider) return;
		const float dur = 0.35f;
		uiFadeT += dt;
		float k = 1f - Mathf.Clamp01(uiFadeT / dur);
		chargeSlider.value = k * chargeSlider.value;
		if (uiFadeT >= dur) { chargeSlider.value = 0; chargeSlider.gameObject.SetActive(false); uiFading = false; }
	}

    // --------------------------- Phase/Debug/HUD ---------------------------
    void ChangePhase(Phase next)
    {
        if (phase == next) return;
        if (logPhaseChanges)
            Debug.Log($"[Fishing] Phase: {phase} -> {next}", this);
        phase = next;
        fsm.Set(next);
    }

    void LateUpdate()
    {
        // 开发热键
        if (devHotkeys)
        {
            if (Input.GetKeyDown(KeyCode.F1)) showDebugOverlay = !showDebugOverlay;
            if (Input.GetKeyDown(KeyCode.F2)) showGraphs = !showGraphs;
            if (Input.GetKeyDown(KeyCode.F3)) reelAnytime = !reelAnytime;
            if (Input.GetKeyDown(KeyCode.F4)) holdRodForward = !holdRodForward;
        }

        // 历史曲线
        EnsureHistoryBuffers();
        float tipSpeed = tipVelFixed.magnitude;
        float bobSpeed = (bobber ? bobber.linearVelocity.magnitude : 0f);
        float dist = (rodTip && bobber) ? Vector2.Distance(rodTip.position, bobber.position) : 0f;
        AppendHistory(Mathf.Max(tipSpeed, bobSpeed), dist);
    }

    void EnsureHistoryBuffers()
    {
        int cap = Mathf.Clamp(historyCapacity, 60, 2048);
        if (speedHistory == null || speedHistory.Length != cap)
        {
            speedHistory = new float[cap];
            distHistory = new float[cap];
            histIndex = 0;
        }
        if (hudStyle == null)
        {
            hudStyle = new GUIStyle();
            hudStyle.fontSize = 12;
            hudStyle.normal.textColor = Color.white;
        }
    }

    void AppendHistory(float speed, float distance)
    {
        if (speedHistory == null) return;
        int cap = speedHistory.Length;
        speedHistory[histIndex] = speed;
        distHistory[histIndex] = distance;
        histIndex = (histIndex + 1) % cap;
    }

    void OnGUI()
    {
        if (!showDebugOverlay) return;
        const float pad = 8f;
        float y = pad;
        float lineH = 18f;
        string p = phase.ToString();
        string txt = $"Phase: {p}\nCasts: {castCount}  MaxLen: {maxCastDistance:F2}\nRelease v: {lastReleaseSpeed:F2}\nTip v: {tipVelFixed.magnitude:F2}\nDist: {(rodTip && bobber ? Vector2.Distance(rodTip.position, bobber.position) : 0f):F2}\nReelAny: {reelAnytime} HoldFwd: {holdRodForward}";
        Vector2 size = hudStyle.CalcSize(new GUIContent(txt));
        GUI.color = new Color(0, 0, 0, 0.6f);
        GUI.Box(new Rect(pad, y, size.x + 2 * pad, size.y + 2 * pad), GUIContent.none);
        GUI.color = Color.white;
        GUI.Label(new Rect(pad * 2, y + pad, size.x, size.y), txt, hudStyle);
        y += size.y + 3 * pad;

        if (showGraphs && Event.current.type == EventType.Repaint && speedHistory != null)
        {
            DrawGraph(new Rect(pad, y, 260, 80), speedHistory, Color.cyan, 0f, 10f);
            y += 90;
            DrawGraph(new Rect(pad, y, 260, 80), distHistory, Color.yellow, 0f, Mathf.Max(1f, maxCast));
            y += 90;
        }
    }

    void DrawGraph(Rect r, float[] data, Color color, float min, float max)
    {
        GUI.color = new Color(0, 0, 0, 0.5f);
        GUI.Box(r, GUIContent.none);
        GUI.color = color;
        if (data == null || data.Length == 0) return;
        int cap = data.Length;
        Vector2 prev = Vector2.zero; bool hasPrev = false;
        for (int i = 0; i < cap; i++)
        {
            int idx = (histIndex + i) % cap;
            float t = i / (float)(cap - 1);
            float v = Mathf.InverseLerp(min, max, data[idx]);
            Vector2 p = new Vector2(Mathf.Lerp(r.x + 2, r.xMax - 2, t), Mathf.Lerp(r.yMax - 2, r.y + 2, v));
            if (hasPrev) Drawing.DrawLine(prev, p, color, 1f);
            prev = p; hasPrev = true;
        }
        GUI.color = Color.white;
    }

    // --------------------------- RodRoot 角度硬限制 ---------------------------
    void EnforceRodRootLimits()
    {
        if (!RodRoot) return;
        // 计算 RodRoot 当前相对 Home 的 Z 角度（-180..180）
        Quaternion rel = Quaternion.Inverse(rodRootHomeRot) * RodRoot.localRotation;
        float zDeg = NormalizeAngle180(rel.eulerAngles.z);
        float clamped = Mathf.Clamp(zDeg, rodMinAngleDeg, rodMaxAngleDeg);
        if (!Mathf.Approximately(clamped, zDeg))
        {
            RodRoot.localRotation = rodRootHomeRot * Quaternion.Euler(0f, 0f, clamped);
        }
    }

    static float NormalizeAngle180(float deg)
    {
        deg %= 360f;
        if (deg > 180f) deg -= 360f;
        if (deg < -180f) deg += 360f;
        return deg;
    }

    // 简单的线段绘制（OnGUI）
    static class Drawing
    {
        static Texture2D _lineTex;
        public static void DrawLine(Vector2 a, Vector2 b, Color color, float width)
        {
            if (_lineTex == null)
            {
                _lineTex = new Texture2D(1, 1, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Repeat };
                _lineTex.SetPixel(0, 0, Color.white);
                _lineTex.Apply();
            }
            Matrix4x4 m = GUI.matrix;
            Color prev = GUI.color;
            GUI.color = color;
            Vector2 d = b - a;
            float ang = Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg;
            float len = d.magnitude;
            GUIUtility.RotateAroundPivot(ang, a);
            GUI.DrawTexture(new Rect(a.x, a.y - width * 0.5f, len, width), _lineTex);
            GUI.matrix = m; GUI.color = prev;
        }
    }

    // --------------------------- 事件容器 ---------------------------
    [System.Serializable]
    public class FishingEvents
    {
        public UnityEvent onChargeStart;
        public UnityEvent onWhipBegin;
        public UnityEvent onRelease;
        public UnityEvent onLand;
        public UnityEvent onReelStart;
        public UnityEvent onReelFinish;
        public UnityEvent onReset;
    }

    // --------------------------- 工具：刚体/关节清理 ---------------------------
    void ZeroRB(Rigidbody2D rb)
    {
        if (!rb) return;
        rb.linearVelocity = Vector2.zero;
        rb.angularVelocity = 0f;
    }

    void ResetHinge(Joint2D j)
    {
        if (!j) return;
        if (j is HingeJoint2D h)
        {
            h.useMotor = false;
            var m = h.motor; m.motorSpeed = 0f; m.maxMotorTorque = 0f; h.motor = m;
            h.useLimits = false;
        }
    }

    // --------------------------- 有限状态机（同文件内隔离） ---------------------------
    [System.Serializable]
    public class PhaseMachine
    {
        public Phase current;
        public void Init(Phase initial) { current = initial; }
        public void Set(Phase next) { current = next; }
        public void Tick(FishingPhysics2D ctx, float dt)
        {
            switch (current)
            {
                case Phase.Idle:
                    if (Input.GetMouseButtonDown(0)) ctx.BeginCharging();
                    break;
                case Phase.Charging:
                    ctx.TickCharging(dt);
                    if (Input.GetMouseButtonUp(0)) ctx.BeginWhip();
                    break;
                case Phase.Whip:
                    ctx.TickWhip(dt);
                    break;
                case Phase.Flight:
                    ctx.TickFlight(dt);
                    ctx.TryBeginReelFromFlight();
                    break;
                case Phase.Landed:
                    if (Input.GetMouseButtonDown(0)) ctx.BeginReel();
                    break;
                case Phase.Reeling:
                    ctx.TickReeling(dt);
                    break;
            }
        }
    }

    // --------------------------- 配置容器 ---------------------------
    [System.Serializable]
    public class FishingConfig
    {
        // Cast
        public float minCast = 1.6f;
        public float maxCast = 7.5f;
        public float chargeSpeed = 1.6f;
        public float flightTime = 0.35f;
        public float landTimeout = 0.45f;
        // Drag
        public float airDrag = 0.4f;
        public float waterDrag = 4.5f;
        // Whip
        public float whipMotorSpeed = 1450f;
        public float whipPulse = 0.14f;
        public bool releaseAtForwardApex = true;
        // Flight
        public float flightGravity = 0.18f;
        public float slackRatio = 0.06f;
        public int tautFramesToLand = 2;
        public float offscreenMargin = 0.05f;
        public bool landOnlyOnWater = true;
        // Launch feel
        public float tipBoost = 1.6f;
        public float extraBoost = 1.1f;
        public float powerCurve = 1.20f;
        // Reel
        public float reelRate = 6.5f;
        public float reelForce = 14f;
        public bool reelAnytime = true;
        // Line
        public int lineSegments = 20;
        public int pixelsPerUnit = 128;
        public int sortingOrder = 10;
        public Color lineColor = new Color(1f, 0.1f, 0.6f, 1f);
        // Rod hold
        public bool holdRodForward = true;
        public float holdForwardDuration = 0.6f;
        public float holdRodBackLimit = 40f;
        public float holdRodForwardLimit = 25f;
    }

    public void ApplyConfig()
    {
        if (config == null) return;
        // Cast
        minCast = config.minCast; maxCast = config.maxCast; chargeSpeed = config.chargeSpeed; flightTime = config.flightTime; landTimeout = config.landTimeout;
        // Drag
        airDrag = config.airDrag; waterDrag = config.waterDrag;
        // Whip
        whipMotorSpeed = config.whipMotorSpeed; whipPulse = config.whipPulse; releaseAtForwardApex = config.releaseAtForwardApex;
        // Flight
        flightGravity = config.flightGravity; slackRatio = config.slackRatio; tautFramesToLand = config.tautFramesToLand; offscreenMargin = config.offscreenMargin; landOnlyOnWater = config.landOnlyOnWater;
        // Launch feel
        tipBoost = config.tipBoost; extraBoost = config.extraBoost; powerCurve = config.powerCurve;
        // Reel
        reelRate = config.reelRate; reelForce = config.reelForce; reelAnytime = config.reelAnytime;
        // Line
        lineSegments = config.lineSegments; pixelsPerUnit = config.pixelsPerUnit; sortingOrder = config.sortingOrder; lineColor = config.lineColor;
        // Rod hold
        holdRodForward = config.holdRodForward; holdForwardDuration = config.holdForwardDuration; holdRodBackLimit = config.holdRodBackLimit; holdRodForwardLimit = config.holdRodForwardLimit;
    }
}
