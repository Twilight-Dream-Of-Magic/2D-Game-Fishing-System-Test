
// =========================================================
// FishingPhysics2D.cs — Unity 6 (6000.x) ）
// =========================================================

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using System;
using System.Collections;
using System.Runtime.InteropServices;

[DisallowMultipleComponent]
[RequireComponent(typeof(LineRenderer))]
public class FishingPhysics2D : MonoBehaviour
{
	[Header("Events（外部可挂回调）")]
	public FishingEvents events = new FishingEvents();

	[Header("Debug & HUD（开发期可视化）")]

	[Tooltip("是否打印阶段切换日志")]
	public bool logPhaseChanges = true;

	// ---------- Inspector（名称保持原样，兼容现有场景） ----------
	[Header("UI & Layers")]
	public Slider chargeSlider;
	public LayerMask waterLayerMask;

	[Header("Rod Chain")]
	public Transform RodRootTransform; // 根节点（旋转枢轴）
	public Rigidbody2D RodSegment1RB, RodSegment2RB, RodSegment3RB, RodTipRB;
	public HingeJoint2D RodSegment1Hinge, RodSegment2Hinge, RodSegment3Hinge;

	[Header("Bobber & Rope")]
	public Rigidbody2D bobberRB;   // 浮标（CircleCollider2D）
	public DistanceJoint2D bobberRopeJoint; // 挂在 bobber 上，connectedBody=rodTip

	// Cast Tuning — removed; replaced by Rope Simple Control params

	[Header("Drag (Unity 6 uses Damping)")]
	public float airDamping = 0.1f;
	public float waterDamping = 4.5f;

	[Header("Reel Tuning")]
	[Tooltip("distance 每秒缩短量")]
	public float reelDistancePerSecond = 10f;

	[Tooltip("沿绳方向额外拉力")]
	public float reelPullForce = 20f;

	[Header("Whip（甩竿）")]
	public float whipMotorAngularSpeed = 1000f;
	public float whipPulseDuration = 0.15f;

	[Header("Whip Animation（根节点动画）")]
	[Tooltip("Whip 阶段是否同时动画 RodRoot 角度")]
	public bool animateRodRootInWhipEnabled = true;

	[Tooltip("Whip 阶段的目标前倾角（度 0~80）")]
	[Range(0, 80)]
	public float maxForwardAngleDegreesInWhip = 60f;

	[Header("Launch Feel（手感增强）")]
	[Tooltip("蓄力曲线 >1 变硬，<1 变软")]
	public float powerCurveExponent = 0.5f;

	[Header("Launch Feel（竿身旋转）")]
	[Tooltip("蓄满时的最大后仰角（度 0~80）")]
	[Range(0, 80)]
	public float maxBackwardAngleDegrees = 60f;

	[Tooltip("Charging 时竿身旋转的平滑时间（秒）")]
	public float chargingRotationSmoothTime = 0.06f;

	[Header("Flight Lock（飞行视觉友好落水）")]
	[Tooltip("飞行允许的额外松弛比例")]
	[Range(0f, 0.5f)]
	public float flightSlackRatio = 0.06f;

	[Tooltip("连续多少帧“已绷紧”就直接落水")]
	[Range(1, 8)]
	public int tautFramesThresholdToLand = 2;

	[Tooltip("离屏外溢容差（0~0.2）")]
	[Range(0f, 0.2f)]
	public float offscreenMarginRatio = 0.05f;

	[Tooltip("只在命中水面时判定落水（演示需求：抛进水里就结束）")]
	public bool landOnlyOnWaterEnabled = false;

	[Header("Water Landing（落水混合逻辑）")]
	[Tooltip("命中水后需要停留的最短时间，避免“刚碰水就落”")]
	[Range(0.1f, 5f)]
	public float waterLandingSettleDelay = 2f;

	[Tooltip("水中锁长的速度阈值，低于则可落")]
	public float waterLockSpeedThreshold = 0.01f;

	[Header("Rope Simple Control（精简线长控制）")]
	[Tooltip("最小绳长度")]
	public float ropeMinLength = 0.01f;

	[Tooltip("最大绳长度")]
	public float ropeMaxLength = 4f;

	[Tooltip("需要至少飞行的时间（秒）")]
	public float ropeMinFlightTime = 1f;

	[Tooltip("飞行超时时长（秒），超过则强制落水")]
	public float ropeFlightMaxTimeout = 4f;

	[Tooltip("解析前向飞行速度（m/s）用于设置鱼竿浮标飞行巡航时的重力系数。") ]
	public float ropeForwardFlightSpeed = 1e+20f;

	[Tooltip("当前是否为无线长度模式（放开上限）")]
	public bool ropeUnlimitedEnabled = false;

	[Tooltip("从最短放到计划长度所需时间（秒）")]
	public float ropeSpoolDuration = 5f;

	[Header("Line Renderer")]
	[Min(2)]
	public int lineRendererSegments = 20;

	[Min(16)]
	public int linePixelsPerUnit = 128;

	public int lineSortingOrder = 10;
	public Color lineColor = new Color(1f, 0.1f, 0.6f, 1f);
	public bool debugDrawLineEnabled = true;

	[Header("Home Pose（复位外观）")]
	public bool snapRodOnReelFinishEnabled = true;
	public bool snapRodOnEmergencyResetEnabled = true;

	[Tooltip("落水时仅复位鱼竿（不回收浮标），用于演示抛进水里即结束")]
	public bool snapRodOnLandEnabled = false;

	[Header("Interaction（交互开关）")]
	[Tooltip("飞行期左键是否可随时强制收线（不等超时/绷紧）")]
	public bool canReelAnytime = true;

	[Header("Rod Forward Hold（抛出后前倾保持）")]
	[Tooltip("抛出后短时间把根节角度限制在前倾区间")]
	public bool holdRodForwardEnabled = true;

	[Tooltip("保持时长（秒）")]
	public float holdForwardDuration = 0.55f;

	[Header("Rod Limits（根节角度限制）")]
	[Tooltip("是否启用根节的角度限制，防止 360° 旋转")]
	public bool RodLimitEnabled = true;

	[Tooltip("根节最小角（度，负为向后）")]
	public float RotRootMinAngleLimit = -60f;

	[Tooltip("根节最大角（度，正为向前）")]
	public float RotRootMaxAngleLimit = 60f;

	[Header("Micro/Cap Lock")]
	[Tooltip("到达cap后是否把浮标设为Kinematic（完全免疫力）")]
	public bool setBobberKinematicAtCap = true;

	[Header("Gravity Configs")]
	[Tooltip("飞行期的重力系数")]
	[Range(0.0001f, 0.4f)]
	public float flightGravityScale = 0.1f;
	[Tooltip("放线期间使用的微重力（不影响放线逻辑）")]
	[Range(0.00001f, 0.01f)]
	public float spoolingMicroGravityScale = 0.01f;

	[Header("Flight Dynamics（飞行分段：起飞/匀速/降落）")]
	[Tooltip("起飞加速期的时长（秒），用正常重力 + 最大空气阻尼")]
	[Range(0.00f, 0.25f)]
	public float flightBoostDuration = 0.05f;

	[Tooltip("起飞加速期的线性阻尼（最大）")]
	[Range(0.00f, 8.0f)]
	public float boostLinearDamping = 0.8f;

	[Tooltip("降落期重力系数的放大倍数（>1 会更快下坠、更有“砸水感”）")]
	[Range(0.5f, 3f)]
	public float descentGravityMultiplier = 1.35f;

	[Tooltip("进入匀速期的最低速度（m/s），起飞时速未达该值会继续视为“起飞加速”")]
	public float cruiseEnterSpeed = 5f;



	// 每次抛投时由蓄力计算出的“本次上限”
	float castMaxLength;

	[Header("Charge Limit")]
	[Tooltip("把蓄力条映射为‘本次抛投的最大线长上限’")]
	public bool useChargeAsMax = true;

	float CurrentMaxCap =>
		ropeUnlimitedEnabled
			? 102400000f
			: (useChargeAsMax ? Mathf.Clamp(castMaxLength, ropeMinLength, ropeMaxLength) : ropeMaxLength);

	[Header("Air Stop At Cap")]
	[Tooltip("飞行中一旦到达本次上限并已绷紧，立刻空中锁绳（关重力）")]
	public bool stopAtCapInAirEnabled = false;

	[Range(1, 10)]
	public int capTautFramesThreshold = 5; // 需要连续绷紧的帧数

	[Header("Air Cap Lock")]
	[Tooltip("到 cap 判定的余量")]
	public float capTolerance = 0.01f;

	[Tooltip("离屏时是否直接空中锁死")]
	public bool forceStopOffscreenEnabled = true;

	// ------------------------- 内部状态 -------------------------
	public enum Phase { Idle, Charging, Whip, Flight, Landed, Reeling }
	Phase currentPhase = Phase.Idle;
	PhaseMachine phaseMachine = new PhaseMachine();

	LineRenderer lineRenderer;
	float ropeFlightTimer, plannedRopeLength;
	float uiFadeElapsed; bool isUiFading;
	float whipElapsedTime;           // 甩竿计时
	int consecutiveTautFrames;                      // 连续“已绷紧”计数
	float chargingSilderValue;
	bool isAirLocked; // 空中锁定后免疫一切外力

	float targetRopeLength;
	float ropeSpoolElapsed;

	enum FlightSubphase { Boost, Cruise, Descent }
	FlightSubphase flightSubphase = FlightSubphase.Boost;
	bool flightSubphaseLockedToDescent; // 进入降落后不回跳，防抖

	// Charging 阶段围绕 RodRoot 的后仰控制（避免 360° 抖动）
	float charingRodBackAngle;
	float charingRotationSmoothVelocityAngle;
	Quaternion RodRootHomeRotation;

	// Home Pose 快照
	struct TPose
	{
		public Vector3 homePosition;
		public Quaternion homeRotation;
		public Vector3 homeScale;

		public TPose(Transform transformPose)
		{
			homePosition = transformPose.localPosition;
			homeRotation = transformPose.localRotation;
			homeScale = transformPose.localScale;
		}

		public void Apply(Transform transformPose)
		{
			transformPose.localPosition = homePosition;
			transformPose.localRotation = homeRotation;
			transformPose.localScale = homeScale;
		}
	}

	TPose BackupPoseRodSegment1, BackupPoseRodSegment2, BackupPoseRodSegment3, BackupPoseRodTip, BackupPoseRodBobber;
	bool isBackupPoseCaptured;

	// Hinge 恢复快照（用于 holdRodForward 临时限制）
	bool hingeHoldActive;
	bool hingePrevUseLimits, hingePrevUseMotor;
	JointAngleLimits2D hingePrevLimits;
	JointMotor2D hingePrevMotor;

	// Debug 运行统计与历史
	bool sawWater;          // 本次飞行是否已首次命中水
	float waterHitTime;        // 首次命中水的相对时间（castTimer）

	// ------------------------- 生命周期 -------------------------
	void Awake()
	{
		lineRenderer = GetComponent<LineRenderer>();
		lineRenderer.useWorldSpace = true;

		if (!lineRenderer.material)
			lineRenderer.material = new Material(Shader.Find("Sprites/Default"));

		lineRenderer.startColor = lineColor;
		lineRenderer.endColor = lineColor;
		lineRenderer.sortingOrder = lineSortingOrder;

		ConfigureLineRenderer();
		phaseMachine.Init(Phase.Idle);

		if (bobberRopeJoint)
		{
			bobberRopeJoint.connectedBody = RodTipRB;
			bobberRopeJoint.autoConfigureDistance = false;
			bobberRopeJoint.maxDistanceOnly = true; // 只限制“最大长度”
			bobberRopeJoint.enabled = false;
		}

		if (bobberRB)
		{
			bobberRB.gravityScale = 0f;
			bobberRB.linearDamping = airDamping;
			bobberRB.angularDamping = 0.05f;
			bobberRB.bodyType = RigidbodyType2D.Kinematic;
			bobberRB.linearVelocity = Vector2.zero; // Unity 6
		}

		if (chargeSlider)
		{
			chargeSlider.minValue = 0f;
			chargeSlider.maxValue = 1f;
			chargeSlider.wholeNumbers = false;
			chargeSlider.value = 0f;
			chargeSlider.gameObject.SetActive(false);
		}

		CaptureBackupPose();
		ResetAll(true);
	}

	void OnValidate()
	{
		if (!Camera.main)
			Debug.LogError("[Fishing] Missing MainCamera (scene needs a Camera tagged 'MainCamera').");
		if (!RodTipRB)
			Debug.LogError("[Fishing] Missing RodTipRB reference.");

		// 可选：把这三行挪回来保持材质/段数同步（不影响运行，仅编辑器观感）
		if (!lineRenderer) lineRenderer = GetComponent<LineRenderer>();
		if (lineRenderer) ConfigureLineRenderer();
	}

	// --------------------------- 主循环 ---------------------------
	void Update()
	{
		currentMousePosition = Input.mousePosition;

		// 右键紧急复位（先彻底取消所有延迟/协程）
		if (Input.GetMouseButtonDown(1))
		{
			CancelInvoke();
			ResetAll(snapRodOnEmergencyResetEnabled);
			return;
		}

		// 把阶段逻辑分发给 fsm（内部仍调用当前类的方法）
		phaseMachine.Tick(this, Time.deltaTime);

		// 对 RodRoot 的角度进行全局硬限制（无论哪个阶段）
		if (RodLimitEnabled)
			EnforceRodRootLimits();

		// Pre-Release：刚性整体
		EnsureAttachedDuringPreRelease();

		TickUiFade(Time.deltaTime);
		UpdateLineRenderer();
		Watchdog();
	}

	private Vector3 currentMousePosition;

	// 判断鼠标相对 RodRoot (鱼竿) 在左(+1)还是右(-1)
	float MouseSideSignOnScreen()
	{
		Vector2 origin = RodRootTransform ? (Vector2)RodRootTransform.position : (Vector2)transform.position;
		Vector2 mouse = Camera.main ? (Vector2)Camera.main.ScreenToWorldPoint(currentMousePosition) : origin + Vector2.right;
		Vector2 delta = mouse - origin;

		delta.Normalize();
		return (delta.x >= 0f) ? +1f : -1f;
	}

	// --------------------------- 状态实现 ---------------------------
	void TickIdle()
	{
		ropeFlightTimer = 0f;

		if (chargeSlider)
		{
			chargeSlider.value = 0f;
			chargeSlider.gameObject.SetActive(true);
		}

		chargingSilderValue = 0f;

		AttachToTipKeepWorld();
		bobberRB.transform.localPosition = Vector3.zero;

		if (bobberRopeJoint)
			bobberRopeJoint.enabled = false;

		ChangePhase(Phase.Charging);

		if (events?.onChargeStart != null)
			events.onChargeStart.Invoke();

		Debug.Log("[Fishing] Charging start", this);
	}

	void TickCharging(float deltaTime)
	{
		// 固定时长：例如 3 秒从 0→1 蓄满 像星露谷）
		const float FILL_DURATION_SECONDS = 3f;
		float fillPerSec = (FILL_DURATION_SECONDS > 1e-6f) ? (1f / FILL_DURATION_SECONDS) : 1f;

		// 先用UI“里”的时间累加
		float nextValue = chargingSilderValue + fillPerSec * deltaTime;

		// 再用 SmoothClamp 做端点平滑（区间 0..1；中段基本等于线性）
		float smoothed = SmoothClamp(nextValue, 0f, 1f, 1e-9f);

		// 写到UI“表”（UI 显示）
		if (chargeSlider)
			chargeSlider.value = smoothed;

		// 立刻“读回表的值”作为权威
		// 即使将来 UI 做了动画/插值，这里也会跟随 UI
		chargingSilderValue = chargeSlider ? Mathf.Clamp01(chargeSlider.value) : smoothed;

		if (chargeSlider)
			chargeSlider.value = chargingSilderValue;

		// 禁用根节马达，Charging 仅用确定性后仰角呈现
		if (RodSegment1Hinge)
			RodSegment1Hinge.useMotor = false;

		// 目标后仰角（按蓄力幅度）
		// t = 蓄力幅度（0..1）
		float ChargeAngleRatio = Mathf.Pow(Mathf.Clamp01(chargingSilderValue), Mathf.Max(1e-10f, powerCurveExponent));

		// 规则：点左边 => 竿向左后仰；点右边 => 竿向右后仰
		// 甩竿时根节的“后仰”动画（仅做外观过渡）
		float targetAngle = ((MouseSideSignOnScreen() < 0f) ? +maxBackwardAngleDegrees : -maxBackwardAngleDegrees) * ChargeAngleRatio;

		if (RodLimitEnabled)
			targetAngle = SmoothClamp(targetAngle, RotRootMinAngleLimit, RotRootMaxAngleLimit, 0.01f);

		charingRodBackAngle = Mathf.SmoothDampAngle(
			charingRodBackAngle,
			targetAngle,
			ref charingRotationSmoothVelocityAngle,
			Mathf.Max(0.0001f, chargingRotationSmoothTime)
		);

		ApplyRootBackAngle(charingRodBackAngle);
	}

	// Charging：计算 plannedLength —— 只按蓄力做线性映射（严格 0..1）
	//无参，松开鼠标时
	void TickCharging()
	{
		whipElapsedTime = 0f;

		float planned01 = Mathf.Clamp01(chargingSilderValue); // 这里就是前面读回的 UI 值
		plannedRopeLength = Mathf.Lerp(ropeMinLength, ropeMaxLength, planned01); //UI长 1% 就是 鱼线长 1%

		if (chargeSlider)
			StartUiFadeOut();
	}

	// 物理步里**直接**用刚体求竿梢速度（旋转→切向速度）
	private bool whipAnimationDone;                   // Update 动画播完后置 true，Fixed 再消化
	private readonly System.Collections.Generic.Queue<Vector2> rodTipPositionHistory = new();
	private readonly System.Collections.Generic.Queue<float> rodTipTimeHistory = new();
	private float rodTipSampleWindowSeconds = 0.070f; // 采样窗口 ~70ms
	private Vector2 ComputeRodTipVelocityFromWindow()
	{
		while (rodTipTimeHistory.Count >= 2)
		{
			float oldestTime = rodTipTimeHistory.Peek();
			float newestTime = rodTipTimeHistory.ToArray()[rodTipTimeHistory.Count - 1];
			if (newestTime - oldestTime > rodTipSampleWindowSeconds) { rodTipTimeHistory.Dequeue(); rodTipPositionHistory.Dequeue(); }
			else break;
		}
		if (rodTipTimeHistory.Count < 2) 
			return Vector2.zero;
		Vector2 oldestPos = rodTipPositionHistory.Peek();
		Vector2 newestPos = rodTipPositionHistory.ToArray()[rodTipPositionHistory.Count - 1];
		float dt = rodTipTimeHistory.ToArray()[rodTipTimeHistory.Count - 1] - rodTipTimeHistory.Peek();
		return (dt > 1e-6f) ? (newestPos - oldestPos) / dt : Vector2.zero;
	}
	private void PushRodTipSample()
	{
		Vector2 tipWorld = (RodTipRB ? (Vector2)RodTipRB.transform.position : (Vector2)transform.position);
		rodTipPositionHistory.Enqueue(tipWorld);
		rodTipTimeHistory.Enqueue(Time.fixedTime); // 物理时间线
	}
	private void ClearRodTipSamples()
	{
		rodTipPositionHistory.Clear();
		rodTipTimeHistory.Clear();
	}
	void FixedUpdate()
	{
		if (RodTipRB)
		{
			PushRodTipSample();
		}
		phaseMachine.FixedTick(this, Time.fixedDeltaTime);
	}

	void TickWhip(float deltaTime)
	{
		//if (whipAnimationDone)
		//{
		//	return;
		//}

		whipElapsedTime += deltaTime;

		// —— 甩竿时根节的“后仰 → 前倾”动画（仅做外观过渡）——
		if (animateRodRootInWhipEnabled && RodRootTransform)
		{
			float whipProgress01 = Mathf.Clamp01(whipElapsedTime / Mathf.Max(0.01f, whipPulseDuration)); // 0..1 进度
			float mouseSideSign = MouseSideSignOnScreen(); // 左(+1)/右(-1)
			float targetRootAngle = Mathf.Lerp(
				charingRodBackAngle, // 当前“后仰角”（充能阶段定下的）
				mouseSideSign * Mathf.Abs(maxForwardAngleDegreesInWhip), // 指向鼠标那一侧的“前倾角”
				whipProgress01
			);

			if (RodLimitEnabled)
				targetRootAngle = Mathf.Clamp(targetRootAngle, RotRootMinAngleLimit, RotRootMaxAngleLimit);

			// 用角度的 SmoothDamp 做顺滑过渡，避免跨 ±180° 抖动
			charingRodBackAngle = Mathf.SmoothDampAngle(
				charingRodBackAngle, targetRootAngle,
				ref charingRotationSmoothVelocityAngle,
				Mathf.Max(0.0001f, chargingRotationSmoothTime)
			);
			ApplyRootBackAngle(charingRodBackAngle);
		}

		if (whipElapsedTime >= whipPulseDuration && !whipAnimationDone)
		{
			whipAnimationDone = true;
		}
	}

	void FixedTickWhip(float fixedDeltaTime)
	{
		Debug.Log($"[Fishing] WHIP chargeSilderValue={chargingSilderValue:F2} plannedRopeLength={plannedRopeLength:F2}", this);

		// —— 把浮标继续“吸附在竿梢”（释放前都是整体）——
		bobberRB.transform.localPosition = Vector3.zero;

		// 动画还没通知完成，继续等
		if (!whipAnimationDone)
			return;

		// —— 动画通知到了：在“物理步”里真正释放 —— //
		whipAnimationDone = false;

		Vector2 targetReleaseDirection =
			(MouseSideSignOnScreen() > 0f) ? Vector2.left : Vector2.right;
		// ★ 用差分窗口估计的竿梢有效速度（Transform 驱动也能抓到）
		Vector2 tipVelocityFromSamples = ComputeRodTipVelocityFromWindow();

		// ★ 若想“二者取大”，可以和 GetPointVelocity 混合，但它在你场景下物理速度常为 0 (由于动画需要改transform的旋转)：
		Vector2 tipVelocityFromRigidBody =
			(RodSegment3RB && RodTipRB) ? RodSegment3RB.GetPointVelocity(RodTipRB.position) : Vector2.zero; // 可保留
		Vector2 tipVelocityForRelease = (tipVelocityFromRigidBody.sqrMagnitude > tipVelocityFromSamples.sqrMagnitude)
									  ? tipVelocityFromRigidBody : tipVelocityFromSamples;

		ReleaseBobberNow(targetReleaseDirection, tipVelocityForRelease);

		ChangePhase(Phase.Flight);
		events?.onRelease?.Invoke();

		flightSubphase = FlightSubphase.Boost;
		flightSubphaseLockedToDescent = false;
		bobberRB.gravityScale = flightGravityScale;
		bobberRB.linearDamping = Mathf.Max(boostLinearDamping, 0f);

		// 竿前倾保持
		if (holdRodForwardEnabled)
			StartCoroutine(HoldRodForwardTemporarily());

		// 清一次窗口，避免旧数据干扰下一次
		ClearRodTipSamples();
	}

	void FixedTickCharging()
	{
		StartCoroutine(PulseHingeMotorTowardMouse()); // 在物理时序下开启“马达脉冲 + 等待固定步时长”的协程

		ChangePhase(Phase.Whip);

		if (events?.onWhipBegin != null)
			events.onWhipBegin.Invoke();

		Debug.Log("[Fishing] Whip start", this);
	}

	IEnumerator PulseHingeMotorTowardMouse()
	{
		if (!RodSegment1Hinge || !RodSegment1RB || !RodTipRB)
			yield break;

		Vector2 targetReleaseDirection = (MouseSideSignOnScreen() > 0f) ? Vector2.left : Vector2.right;

		// 当前轴向：第一节枢轴 -> 竿梢（单位向量）
		Vector2 currentAxisDirection = ((Vector2)RodTipRB.position - (Vector2)RodSegment1RB.position).normalized;

		float sign = (Vector2.SignedAngle(currentMousePosition, targetReleaseDirection) > 0f) ? 1f : -1f;

		// prepare motor (JointMotor2D is a struct -> write back)
		var motor = RodSegment1Hinge.motor;
		motor.motorSpeed = sign * Mathf.Abs(whipMotorAngularSpeed); // degrees/sec
		motor.maxMotorTorque = 1e10f; // maxMotorTorquePulse configurable 
		RodSegment1Hinge.motor = motor;
		RodSegment1Hinge.useMotor = true;

		// clamp duration
		float duration = Mathf.Clamp(whipPulseDuration, 0.0001f, 5f);

		// Wait in physics ticks until accumulated fixed time >= duration
		float elapsed = 0f;
		while (elapsed < duration)
		{
			// accumulate the fixed-step time
			elapsed += Time.fixedDeltaTime;
			yield return new WaitForFixedUpdate();
		}

		// turn motor off / restore safe defaults
		motor = RodSegment1Hinge.motor;
		motor.motorSpeed = 0f;
		motor.maxMotorTorque = 0f;
		RodSegment1Hinge.motor = motor;
		RodSegment1Hinge.useMotor = false;

		yield break;
	}

	// 统一设置：释放时线状态
	void SetRopeForRelease()
	{
		if (!bobberRopeJoint)
			return;
		bobberRopeJoint.enabled = true;

		// 飞行：用“最大长度模式”，上限 = 本次上限（不突破）
		bobberRopeJoint.maxDistanceOnly = true;

		// 起步从最短（或本次上限更小者）
		bobberRopeJoint.distance = Mathf.Min(ropeMinLength, CurrentMaxCap);

		// 放线动画朝“本次上限”推进（不再用 >cap 的 slack）
		ropeSpoolElapsed = 0f;
	}


	// 只负责：把浮标打出去「水平很强（左右明确）+ 上抛合理」，必要时叠加竿梢“味道”
	void ReleaseBobberNow(Vector2 directionHint, Vector2 rodTipVelocityWorld)
	{
		// ===================== 可调常量（仅本函数内） =====================
		// —— 水平（左右）——
		// “按计划线长换算”的水平速度斜率（低蓄力→满蓄力），单位：(m/s) / 计划米数
		const float horizontalSpeedPerMeterAtLowCharge = 5.0f;   // 蓄力=0
		const float horizontalSpeedPerMeterAtFullCharge = 50.0f;  // 蓄力=1  想更远就加大
		const float horizontalSpeedFloor = 1.0f;   // 无论如何至少这么快（m/s）

		// 竿梢“前向味道”混入（只取世界左右轴上的投影）
		const float tipForwardAdditiveScale = 0.75f;  // 正向加这么多倍
		const float tipForwardOppositePreserveRatio = 0.20f;  // 反向只保留

		// —— 竖直（上抛）——
		// 用“期望最高点”反解 v0y：h = v0y^2/(2g)
		const float apexLowChargeMeters = 1f;   // 蓄力=0 的顶点高度
		const float apexFullChargeMeters = 5f;   // 蓄力=1 的顶点高度
		const float apexBlendExponent = 0.75f;  // 0..∞（>1 偏硬）
		const float tipUpwardCarryScale = 0.6f;  // 叠加竿梢向上的那一部分

		// —— 形状保护 —— 
		const float minimumLaunchAngleDegrees = 20f;    // 兜底最小仰角，防“太扁”
		const float maximumLaunchSpeedMetersPerSecond = 200f;    // 软上限，防爆表

		bobberRopeJoint.enabled = false;

		DetachFromTipKeepWorld();

		// ===================== 基础量 =====================
		float charge01 = Mathf.Clamp01(chargingSilderValue);
		float effectiveGravity = Mathf.Abs(Physics2D.gravity.y) * Mathf.Max(1e-6f, flightGravityScale);

		// 明确的世界左右单位向量（和你 MouseSideSignOnScreen 逻辑一致）
		Vector2 worldHorizontalUnit = (MouseSideSignOnScreen() > 0f) ? Vector2.left : Vector2.right;

		// ===================== 先定“水平速度” =====================
		// 按计划线长换算出目标水平速度（越想甩远 → 越大）
		float horizontalPerMeter = Mathf.Lerp(
			horizontalSpeedPerMeterAtLowCharge,
			horizontalSpeedPerMeterAtFullCharge,
			charge01
		);
		float horizontalSpeedMagnitude = Mathf.Max(
			horizontalSpeedFloor,
			plannedRopeLength * horizontalPerMeter
		);

		// 叠加竿梢在“世界左右轴”的投影（只取少量反向，避免过零没劲）
		float tipForwardAlongHorizontal = Vector2.Dot(rodTipVelocityWorld, worldHorizontalUnit);
		if (tipForwardAlongHorizontal < 0f)
			tipForwardAlongHorizontal *= tipForwardOppositePreserveRatio;
		horizontalSpeedMagnitude += tipForwardAlongHorizontal * tipForwardAdditiveScale;

		// 水平分量向量（只沿左右，不改变方向）
		Vector2 horizontalComponent = worldHorizontalUnit * horizontalSpeedMagnitude;

		// ===================== 再定“竖直速度” =====================
		// 目标顶点高度随蓄力插值，然后反解 v0y
		float desiredApexMeters = Mathf.Lerp(
			apexLowChargeMeters,
			apexFullChargeMeters,
			Mathf.Pow(charge01, apexBlendExponent)
		);
		float verticalSpeedMagnitude = Mathf.Sqrt(Mathf.Max(0f, 2f * effectiveGravity * desiredApexMeters));

		// 叠加竿梢向上的那一部分
		if (rodTipVelocityWorld.y > 0f)
			verticalSpeedMagnitude += rodTipVelocityWorld.y * tipUpwardCarryScale;

		Vector2 verticalComponent = Vector2.up * verticalSpeedMagnitude;

		// ===================== 合成初速度（世界系正交分解） =====================
		Vector2 launchVelocity = horizontalComponent + verticalComponent;

		// —— 兜底最小仰角（不改变水平大小，只抬竖直到最小角）——
		float currentHorizontalMag = Mathf.Abs(Vector2.Dot(launchVelocity, worldHorizontalUnit));
		float currentVerticalMag = Mathf.Max(0f, Vector2.Dot(launchVelocity, Vector2.up));
		float minVy = currentHorizontalMag * Mathf.Tan(minimumLaunchAngleDegrees * Mathf.Deg2Rad);
		if (currentVerticalMag < minVy)
		{
			currentVerticalMag = minVy;
			launchVelocity = worldHorizontalUnit * currentHorizontalMag + Vector2.up * currentVerticalMag;
		}

		// 速度软上限
		launchVelocity = Vector2.ClampMagnitude(launchVelocity, maximumLaunchSpeedMetersPerSecond);

		// ===================== 真正写回 & 进入飞行 =====================
		SetRopeForRelease();
		bobberRB.linearDamping = airDamping;
		bobberRB.gravityScale = flightGravityScale;
		bobberRB.linearVelocity = launchVelocity;

		// 调试日志
		float angleDeg = Mathf.Atan2(launchVelocity.y, Vector2.Dot(launchVelocity, worldHorizontalUnit)) * Mathf.Rad2Deg;
		Debug.LogWarning($"[Fishing] RELEASE |v|={launchVelocity.magnitude:F2}  angle={angleDeg:F1}°  " +
						 $"vx={currentHorizontalMag:F2} vy={currentVerticalMag:F2}", this);

		// 放线与状态维持（沿用你原来的）
		castMaxLength = Mathf.Clamp(plannedRopeLength, ropeMinLength, ropeMaxLength);
		ropeFlightTimer = 0f; consecutiveTautFrames = 0; sawWater = false; waterHitTime = 0f;
	}

	public struct FlightingBaseData
	{
		public float cap;        // 本次上限（已考虑 charge/unlimited）
		public float tipTo;      // 竿梢->浮标 的距离
		public bool isTight;       // 是否已绷紧
		public bool inWaterNow;  // 当前位置是否在水层
	}

	// 类内新增一个缓存字段（同一帧内跨函数复用）
	private FlightingBaseData cachedFlightBase;

	FlightingBaseData BuildFlightingBaseData()
	{
		var data = new FlightingBaseData();

		// 统一用 Unity 6 的 API（linearVelocity / physics 2D）
		data.cap = CurrentMaxCap;

		// tip->bobber 的实时数据
		float tipTo = 0f;
		bool isTight = false;

		if (RodTipRB && bobberRB)
		{
			tipTo = Vector2.Distance(RodTipRB.position, bobberRB.position);
			isTight = bobberRopeJoint
				? (tipTo >= (bobberRopeJoint.distance - Mathf.Abs(capTolerance)))
				: false;
		}

		data.tipTo = tipTo;
		data.isTight = isTight;

		// 图层检测：水体（你已经有 waterLayerMask）
		data.inWaterNow = Physics2D.OverlapPoint(bobberRB.position, waterLayerMask) != null;

		return data;
	}

	// —— 飞行姿态：更远的巡航、更清晰的抛物线 —— //
	void FixedTickFlight(float fixedDeltaTime)
	{
		// ========= 可调常量（仅此处） =========
		// —— 放线 —— 
		const float ropeSpoolOvershootSlackRatio = 0.2f;   // 把 distance 向 cap 推进时附带的视觉松弛（占 cap 百分比）
		const float ropeSpoolHardEpsilon = 0.003f;          // 判定“仍在硬放线”的余量

		// —— 子阶段门槛 —— 
		const float minBoostDuration = 0.06f;               // 起飞段最短维持时间（秒）
		const float minCruiseEnterSpeed = 5.0f;             // 进入巡航的最低速度（m/s）
		const float descentEnterDownSpeed = 5.0f;           // 进入降落的向下速度阈值（m/s，vy <= -阈值）

		// —— 三段重力 & 阻尼（Unity 6：gravityScale / linearDamping）——
		const float gravityScaleBoost = 0.2f;              // 起飞：用 flightGravityScale × 该倍数
		const float linearDampingBoost = 0.55f;             // 起飞：较大阻尼，快速“立弧”

		const float gravityScaleCruiseMicro = 0.001f;        // 巡航：极低重力（拉长水平航程）
		const float linearDampingCruise = 0.01f;            // 巡航：低阻尼，尽量保速

		const float gravityScaleDescentMultiplier = 0.6f;  // 降落：flightGravityScale × 该倍数（更有“砸水感”）
		const float linearDampingDescent = 0.05f;           // 降落：最小阻尼，避免拖死

		// —— Offscreen 保护 —— 
		const float offscreenStopMarginViewport = 0.05f;    // 视口外余量，超出直接空锁

		// ========= 基础与早退 =========
		if (!bobberRB || !bobberRopeJoint || !RodTipRB) 
			return;

		// —— 记录：是否首次命中水（仅用于“水面稳定”判定，不在放线中直接落水）——
		bool isInWaterNow = Physics2D.OverlapPoint(bobberRB.position, waterLayerMask) != null;
		if (isInWaterNow && !sawWater) 
		{ 
			sawWater = true; 
			waterHitTime = ropeFlightTimer; 
		}

		// —— 离屏保护：直接空中锁 ——
		if (forceStopOffscreenEnabled && IsOffscreen(bobberRB.position, offscreenStopMarginViewport))
		{
			FixedTickCheckedLanded(1);
			return;
		}

		cachedFlightBase = BuildFlightingBaseData();

		// —— 线长推进（演出：distance 只增不减，且不超过 cap）——
		if (bobberRopeJoint.maxDistanceOnly)
		{
			// 快速推进 distance → cap（首秒快，之后趋缓）
			ropeSpoolElapsed += fixedDeltaTime;
			float t = Mathf.Clamp01(ropeSpoolElapsed / Mathf.Max(0.01f, ropeSpoolDuration));
			// 平滑到 cap，并留一点视觉“松弛”
			float target = Mathf.Min(cachedFlightBase.cap, Mathf.Lerp(bobberRopeJoint.distance, cachedFlightBase.cap, t));
			float slack = cachedFlightBase.cap * ropeSpoolOvershootSlackRatio;
			float newDistance = Mathf.Max(bobberRopeJoint.distance, Mathf.Min(cachedFlightBase.cap, target + slack));
			bobberRopeJoint.distance = newDistance;
		}

		// —— 基础量 —— 
		ropeFlightTimer += fixedDeltaTime;
		cachedFlightBase.isTight = cachedFlightBase.tipTo >= bobberRopeJoint.distance - 0.01f;
		if (cachedFlightBase.isTight) 
			consecutiveTautFrames++; 
		else 
			consecutiveTautFrames = 0;

		// ========= 子阶段切换（Boost → Cruise → Descent） =========
		// 未锁死前允许从 Boost 进入 Cruise；一旦进 Descent 就锁定不回跳
		if (!flightSubphaseLockedToDescent)
		{
			if (flightSubphase == FlightSubphase.Boost)
			{
				bool timeOk = ropeFlightTimer >= Mathf.Max(minBoostDuration, flightBoostDuration);
				bool speedOk = bobberRB.linearVelocity.magnitude >= Mathf.Max(minCruiseEnterSpeed, cruiseEnterSpeed);
				if (timeOk && speedOk)
					flightSubphase = FlightSubphase.Cruise;
			}
			else if (flightSubphase == FlightSubphase.Cruise)
			{
				bool goingDownFast = bobberRB.linearVelocity.y <= -Mathf.Abs(descentEnterDownSpeed);
				bool waterSettle = sawWater && (ropeFlightTimer - waterHitTime) >= Mathf.Max(0f, waterLandingSettleDelay);
				if (goingDownFast || (cachedFlightBase.tipTo >= cachedFlightBase.cap - capTolerance) || waterSettle)
				{
					flightSubphase = FlightSubphase.Descent;
					flightSubphaseLockedToDescent = true;
				}
			}
		}

		// ========= 三段姿态：设置 gravityScale / linearDamping =========
		switch (flightSubphase)
		{
			case FlightSubphase.Boost:
				bobberRB.gravityScale = flightGravityScale * gravityScaleBoost;
				bobberRB.linearDamping = Mathf.Max(linearDampingBoost, 0f);
				// 起飞头几帧直接 return，给“起弧”时间
				if (ropeFlightTimer < Mathf.Min(0.08f, Mathf.Max(0.02f, flightBoostDuration))) 
					return;
				break;

			case FlightSubphase.Cruise:
				{
					// 巡航：若仍在硬放线，按速度映射到 micro 重力，速度越大越接近极小重力
					bool spoolingHard = bobberRopeJoint.maxDistanceOnly && (bobberRopeJoint.distance < cachedFlightBase.cap - ropeSpoolHardEpsilon);
					if (spoolingHard)
					{
						float speed = bobberRB.linearVelocity.magnitude;
						float speedLow = Mathf.Max(0.5f, ropeForwardFlightSpeed * 0.25f);
						float speedHigh = Mathf.Max(1.0f, ropeForwardFlightSpeed * 0.90f);
						float lerp01 = Mathf.Clamp01((speed - speedLow) / Mathf.Max(0.001f, (speedHigh - speedLow)));
						float smooth = Smooth01C3(lerp01); // 你类里已有的C³平滑
						float g = Mathf.Lerp(flightGravityScale, Mathf.Max(0f, gravityScaleCruiseMicro), smooth);
						bobberRB.gravityScale = g;
					}
					else
					{
						bobberRB.gravityScale = Mathf.Max(0f, gravityScaleCruiseMicro);
					}
					bobberRB.linearDamping = Mathf.Max(linearDampingCruise, 0f);
					break;
				}

			case FlightSubphase.Descent:
				bobberRB.gravityScale = flightGravityScale * Mathf.Max(0.01f, descentGravityMultiplier * gravityScaleDescentMultiplier);
				bobberRB.linearDamping = Mathf.Max(linearDampingDescent, 0f);
				break;
		}
	}


	bool ShouldForceLand(bool inWaterNow, bool tight, bool inward, bool slowInWater)
	{
		bool timeout = ropeFlightTimer >= ropeFlightMaxTimeout;

		if (landOnlyOnWaterEnabled)
		{
			if (!inWaterNow)
				return false;

			bool settleOk = sawWater && (ropeFlightTimer - waterHitTime) >= Mathf.Max(0f, waterLandingSettleDelay);
			// 只在“绷紧或超时”并且水面稳定后才落水
			return settleOk && (tight || timeout);
		}
		else
		{
			return (tight && ropeFlightTimer >= ropeMinFlightTime) || timeout;
		}
	}

	int FixedTickCheckLanded(in FlightingBaseData cacheData)
	{
		// 早退：组件未就绪
		if (!bobberRB || !bobberRopeJoint || !RodTipRB)
			return 0;

		// 空中锁：到达 cap 且满足最小飞行时长
		// 在 FixedTickCheckLanded 的 “空中锁”分支里替换为：
		if (stopAtCapInAirEnabled &&
			cacheData.tipTo >= cacheData.cap - Mathf.Abs(capTolerance) &&
			ropeFlightTimer >= ropeMinFlightTime &&
			bobberRB.linearVelocity.y <= 0f &&                         // 必须进入下降段
			consecutiveTautFrames >= Mathf.Max(1, capTautFramesThreshold)) // 连续绷紧N帧
		{
			return 1; // 空中锁
		}

		// 非水早落水（允许时）：绷紧达到阈值且飞行时间达标
		if (!landOnlyOnWaterEnabled && consecutiveTautFrames >= Mathf.Max(1, tautFramesThresholdToLand) && ropeFlightTimer >= ropeMinFlightTime)
			return 2;

		// 水面稳定落水：仍然需要 inward/slowInWater（它们只在这里用，算一次即可）
		bool inward = Vector2.Dot(RodTipRB.position - bobberRB.position, bobberRB.linearVelocity) > 0f || bobberRB.linearVelocity.sqrMagnitude < 0.0003f;
		bool slowInWater = bobberRB.linearVelocity.sqrMagnitude <= waterLockSpeedThreshold * waterLockSpeedThreshold;

		if (ShouldForceLand(cacheData.inWaterNow, cacheData.isTight, inward, slowInWater))
			return 2;

		// 允许随时收线
		if (canReelAnytime)
			return 3;

		return 0;
	}

	void FixedTickCheckedLanded(int state)
	{
		if (state == 1)
		{
			AirLockAtCap();
			ChangePhase(Phase.Landed);
			if (events?.onLand != null)
				events.onLand.Invoke();

			Debug.Log($"[Fishing] AIR-LOCK cap={cachedFlightBase.tipTo:F2} tipTo={cachedFlightBase.cap:F2}", this);
		}
		else if (state == 2)
		{
			LockRopeAtCurrentAndWaterDamping();

			ChangePhase(Phase.Landed);
			if (events?.onLand != null)
				events.onLand.Invoke();

			if (snapRodOnLandEnabled)
				RestoreRodPoseOnly();

			Debug.Log($"[Fishing] LANDED lockLength={Vector2.Distance(RodTipRB.position, bobberRB.position):F2}", this);
		}
		else if (state == 3)
		{
			LockRopeAtCurrentAndWaterDamping();
			FixedTickLanded();
			Debug.Log("[Fishing] To reel from Flight (anytime during spooling)", this);
		}
	}

	bool IsOffscreen(Vector3 worldPos, float margin)
	{
		if (!Camera.main)
			return false;

		var v = Camera.main.WorldToViewportPoint(worldPos);
		return v.x < -margin || v.x > 1f + margin || v.y < -margin || v.y > 1f + margin;
	}

	void AirLockAtCap()
	{
		if (!bobberRB || !RodTipRB) return;

		float tipTo = Vector2.Distance(RodTipRB.position, bobberRB.position);
		float clamped = Mathf.Clamp(tipTo, ropeMinLength, CurrentMaxCap);

		// 固定在当前长度；视觉上仍能画线
		if (bobberRopeJoint)
		{
			bobberRopeJoint.enabled = false; // 彻底断开物理约束，避免再牵动
			bobberRopeJoint.maxDistanceOnly = false; // 无所谓，但保持一致
			bobberRopeJoint.distance = clamped; // 只留作记录
		}

		// 完全免疫一切力
		bobberRB.linearVelocity = Vector2.zero;
		bobberRB.angularVelocity = 0f;
		bobberRB.gravityScale = 0f;
		bobberRB.linearDamping = airDamping; // 维持空气阻尼（不是水）
		bobberRB.bodyType = RigidbodyType2D.Kinematic;

		isAirLocked = true;
	}

	void LockRopeAtCurrentAndWaterDamping()
	{
		float currentDistance = Vector2.Distance(RodTipRB.position, bobberRB.position);
		// 落水后的实际长度 ∈ [min, 本次上限]
		float clamped = Mathf.Clamp(currentDistance, ropeMinLength, CurrentMaxCap);

		Debug.Log($"[Fishing] LANDED rawtDistance={currentDistance:F2} clamptDistance={clamped:F2} capLength={CurrentMaxCap:F2}", this);

		if (bobberRopeJoint)
		{
			bobberRopeJoint.enabled = true;
			bobberRopeJoint.maxDistanceOnly = false; // 固定精确长度
			bobberRopeJoint.distance = clamped;
		}

		bobberRB.linearVelocity = Vector2.zero;
		bobberRB.angularVelocity = 0f;
		bobberRB.linearDamping = waterDamping;
		bobberRB.gravityScale = 0f;
	}

	void FixedTickLanded()
	{
		if (isAirLocked)
		{
			// 重新接回物理世界
			bobberRB.bodyType = RigidbodyType2D.Dynamic;

			if (bobberRopeJoint)
			{
				bobberRopeJoint.enabled = true;
				bobberRopeJoint.maxDistanceOnly = false; // 收线是“绝对距离”模式
				bobberRopeJoint.distance = Vector2.Distance(RodTipRB.position, bobberRB.position);
			}

			isAirLocked = false;
		}

		if (bobberRopeJoint)
			bobberRopeJoint.maxDistanceOnly = false; // 绝对长度模式：distance 变短会“拖回来”

		ChangePhase(Phase.Reeling);
		if (events?.onReelStart != null)
			events.onReelStart.Invoke();
		Debug.Log("[Fishing] Reel start", this);
	}

	void FixedTickReeling(float fixedDeltaTime)
	{
		if (bobberRopeJoint)
			bobberRopeJoint.distance = Mathf.Max(0f, bobberRopeJoint.distance - Mathf.Max(0f, reelDistancePerSecond) * fixedDeltaTime);

		if (RodTipRB && bobberRB)
		{
			Vector2 toTip = (Vector2)RodTipRB.position - bobberRB.position;
			if (toTip.sqrMagnitude > float.Epsilon)
				bobberRB.AddForce(toTip.normalized * reelPullForce, ForceMode2D.Force);

			if (toTip.magnitude <= 0.12f || (bobberRopeJoint && bobberRopeJoint.distance <= 0.02f))
			{
				if (bobberRopeJoint)
				{
					bobberRopeJoint.enabled = false;
					bobberRopeJoint.distance = 0f;
				}

				AttachToTipKeepWorld();
				bobberRB.transform.localPosition = Vector3.zero;
				bobberRB.gravityScale = 0f;

				if (snapRodOnReelFinishEnabled)
					RestoreHomePoseInstant();

				ChangePhase(Phase.Idle);

				if (events?.onReelFinish != null)
					events.onReelFinish.Invoke();

				Debug.Log("[Fishing] Reel finish", this);
			}
		}
	}

	// --------------------------- 数学 / Math --------------------------------

	// C³ 连续的七次 smoothstep：S3(t) = -20 t^7 + 70 t^6 - 84 t^5 + 35 t^4
	static float Smooth01C3(float value)
	{
		value = Mathf.Clamp01(value);
		float t2 = value * value;
		float t3 = t2 * value;
		float t4 = t3 * value;
		float t5 = t4 * value;
		float t6 = t5 * value;
		float t7 = t6 * value;
		return -20f * t7 + 70f * t6 - 84f * t5 + 35f * t4;
	}

	/// <summary>
	/// 在[min, max]两端各留一段margin作柔和过渡；
	/// 中间段恒等映射（保证区间内任意浮点可取到）；
	/// 过渡段使用C²连续的五次Hermite平滑；
	/// 区间外仍会被夹到端点（但过渡到端点是平滑的）。
	/// </summary>
	public float SmoothClamp(float value, float min, float max, float margin)
	{
		if (min > max)
		{
			var temp = min;
			min = max;
			max = temp;
		}

		margin = Mathf.Max(0f, margin);

		//没有缓冲过渡带是退化为普通的Clamp
		if (margin <= 0f || (max - min) <= float.Epsilon)
		{
			return Mathf.Clamp(value, min, max);
		}

		float leftStart = min - margin;
		float leftEnd = min + margin;
		float rightStart = max - margin;
		float rightEnd = max + margin;

		if (value <= leftStart)
			return min; //远左
		if (value < leftEnd)
		{
			//远左 ~ 近左
			float f1 = (value - leftStart) / (leftEnd - leftStart); // 0..1
			float f2 = Smooth01C3(f1);
			// 过渡到恒等映射
			return Mathf.Lerp(min, value, f2);
		}

		if (value <= rightStart)
			return value; //中间
		if (value < rightEnd)
		{
			//中间 ~ 近右
			float f1 = (value - rightStart) / (rightEnd - rightStart); // 0..1
			float f2 = Smooth01C3(f1);
			// 过渡到恒等映射
			return Mathf.Lerp(value, max, f2);
		}

		return max; //远右
	}

	// --------------------------- 复位 / Home Pose ---------------------------
	void CaptureBackupPose()
	{
		if (isBackupPoseCaptured || RodSegment1RB == null || RodSegment2RB == null || RodSegment3RB == null || RodTipRB == null || bobberRB == null)
			return;

		BackupPoseRodSegment1 = new TPose(RodSegment1RB.transform);
		BackupPoseRodSegment2 = new TPose(RodSegment2RB.transform);
		BackupPoseRodSegment3 = new TPose(RodSegment3RB.transform);
		BackupPoseRodTip = new TPose(RodTipRB.transform);
		BackupPoseRodBobber = new TPose(bobberRB.transform);

		isBackupPoseCaptured = true;

		if (RodRootTransform)
			RodRootHomeRotation = RodRootTransform.localRotation;
	}

	void RestoreHomePoseInstant()
	{
		if (!isBackupPoseCaptured) return;

		var b1 = RodSegment1RB.bodyType;
		var b2 = RodSegment2RB.bodyType;
		var b3 = RodSegment3RB.bodyType;

		RodSegment1RB.bodyType = RodSegment2RB.bodyType = RodSegment3RB.bodyType = RigidbodyType2D.Kinematic;

		BackupPoseRodSegment1.Apply(RodSegment1RB.transform);
		BackupPoseRodSegment2.Apply(RodSegment2RB.transform);
		BackupPoseRodSegment3.Apply(RodSegment3RB.transform);
		BackupPoseRodTip.Apply(RodTipRB.transform);

		RodSegment1RB.bodyType = b1;
		RodSegment2RB.bodyType = b2;
		RodSegment3RB.bodyType = b3;

		if (RodRootTransform)
			RodRootTransform.localRotation = RodRootHomeRotation;

		AttachToTipKeepWorld();
		bobberRB.transform.localPosition = Vector3.zero;
	}

	// 仅复位鱼竿，不处理浮标（用于落水即结束的展示效果）
	void RestoreRodPoseOnly()
	{
		if (!isBackupPoseCaptured) return;

		var b1 = RodSegment1RB.bodyType;
		var b2 = RodSegment2RB.bodyType;
		var b3 = RodSegment3RB.bodyType;

		RodSegment1RB.bodyType = RodSegment2RB.bodyType = RodSegment3RB.bodyType = RigidbodyType2D.Kinematic;

		BackupPoseRodSegment1.Apply(RodSegment1RB.transform);
		BackupPoseRodSegment2.Apply(RodSegment2RB.transform);
		BackupPoseRodSegment3.Apply(RodSegment3RB.transform);
		BackupPoseRodTip.Apply(RodTipRB.transform);

		RodSegment1RB.bodyType = b1;
		RodSegment2RB.bodyType = b2;
		RodSegment3RB.bodyType = b3;
	}

	void ResetAll(bool snapRod)
	{
		// —— 强制进入 Idle，并把 FSM 重新初始化，切断上一阶段的 Tick —— //
		ChangePhase(Phase.Idle);
		phaseMachine.Init(Phase.Idle);

		// —— 取消任何延迟/Invoke/协程（StopHingeMotor、ReleaseHoldRodForward 等）—— //
		CancelInvoke();
		StopAllCoroutines();

		hingeHoldActive = false; // 防止误判

		// —— 关闭所有关节的 motor/limits —— //
		ResetHinge(RodSegment1Hinge);
		ResetHinge(RodSegment2Hinge);
		ResetHinge(RodSegment3Hinge);

		// —— 基本标量清零 —— //
		ropeFlightTimer = 0f;
		plannedRopeLength = 0f;

		isUiFading = false;
		uiFadeElapsed = 0f;

		whipElapsedTime = 0f;
		consecutiveTautFrames = 0;

		castMaxLength = 0f;
		isAirLocked = false;

		chargingSilderValue = 0f;

		whipAnimationDone = false;
		ClearRodTipSamples();

		// —— 线关节：彻底关闭并清零 —— //
		if (bobberRopeJoint)
		{
			bobberRopeJoint.enabled = false;
			bobberRopeJoint.maxDistanceOnly = true;
			bobberRopeJoint.distance = 0f;
		}

		// —— 清零所有刚体的线/角速度（不仅仅是 bobber）—— //
		ZeroRB(RodSegment1RB);
		ZeroRB(RodSegment2RB);
		ZeroRB(RodSegment3RB);
		ZeroRB(RodTipRB);
		ZeroRB(bobberRB);

		// —— 复位外观（先把段设为 Kinematic，套用快照，再还原）—— //
		if (snapRod)
		{
			RestoreHomePoseInstant();
		}
		else
		{
			// 即便不 snapRod，也要把 bobber 正确回挂
			AttachToTipKeepWorld();
			bobberRB.transform.localPosition = Vector3.zero;
		}

		// —— Bobber 回到“整体状态” —— //
		if (bobberRB)
		{
			bobberRB.bodyType = RigidbodyType2D.Kinematic;
			bobberRB.linearVelocity = Vector2.zero;
			bobberRB.angularVelocity = 0f;
			bobberRB.linearDamping = airDamping;
			bobberRB.gravityScale = 0f;

			AttachToTipKeepWorld();
			bobberRB.transform.localPosition = Vector3.zero;
		}

		// —— UI & 线渲染器 —— //
		if (chargeSlider)
		{
			chargeSlider.value = 0f;
			chargeSlider.gameObject.SetActive(false);
		}

		if (lineRenderer)
		{
			lineRenderer.enabled = false;
			lineRenderer.positionCount = Mathf.Max(2, lineRendererSegments) + 1;

			Vector3 p = RodTipRB ? (Vector3)RodTipRB.position : transform.position;
			for (int i = 0; i < lineRenderer.positionCount; i++)
				lineRenderer.SetPosition(i, p);
		}

		if (events?.onReset != null)
			events.onReset.Invoke();

		Debug.Log("[Fishing] ResetToIdle", this);
	}

	// --------------------------- 附着/解绑 ---------------------------
	void AttachToTipKeepWorld()
	{
		var t = bobberRB.transform;
		t.SetParent(RodTipRB.transform, true); // 保持世界位姿
		t.position = RodTipRB.position;        // 再贴紧
		t.rotation = RodTipRB.transform.rotation;
		t.localScale = BackupPoseRodBobber.homeScale;     // 防“越抛越大/小”

		bobberRB.bodyType = RigidbodyType2D.Kinematic;
		bobberRB.linearVelocity = Vector2.zero;
		bobberRB.angularVelocity = 0f;
		bobberRB.linearDamping = airDamping;
	}

	void DetachFromTipKeepWorld()
	{
		var t = bobberRB.transform;
		t.SetParent(null, true);             // 释放瞬间解绑
		bobberRB.bodyType = RigidbodyType2D.Dynamic;
	}

	void EnsureAttachedDuringPreRelease()
	{
		if (currentPhase == Phase.Idle || currentPhase == Phase.Charging || currentPhase == Phase.Whip)
		{
			if (bobberRB.transform.parent != RodTipRB.transform)
				AttachToTipKeepWorld();

			bobberRB.transform.localPosition = Vector3.zero;
		}
	}

	// 围绕 RodRoot 旋转
	void ApplyRootBackAngle(float backAngle)
	{
		if (!RodRootTransform) 
			return;

		// 硬限制角度范围
		if (RodLimitEnabled)
			backAngle = Mathf.Clamp(backAngle, RotRootMinAngleLimit, RotRootMaxAngleLimit);

		// 基于 RodRoot 的 home 旋转
		RodRootTransform.localRotation = RodRootHomeRotation * Quaternion.Euler(0f, 0f, backAngle);
	}

	// --------------------------- 竿前倾保持（真正的窄窗口 + 干净收尾） ---------------------------
	IEnumerator HoldRodForwardTemporarily()
	{
		if (!RodSegment1Hinge || hingeHoldActive)
			yield break;

		// 备份
		hingePrevUseLimits = RodSegment1Hinge.useLimits;
		hingePrevUseMotor = RodSegment1Hinge.useMotor;
		hingePrevLimits = RodSegment1Hinge.limits;
		hingePrevMotor = RodSegment1Hinge.motor;

		// 局部工具：尽力恢复（如果铰链还活着）
		void RestoreIfPossible()
		{
			if (RodSegment1Hinge)
			{
				RodSegment1Hinge.useLimits = hingePrevUseLimits;
				RodSegment1Hinge.limits = hingePrevLimits;
				RodSegment1Hinge.useMotor = hingePrevUseMotor;
				RodSegment1Hinge.motor = hingePrevMotor;
			}
			hingeHoldActive = false;
		}

		// 设置“窄窗口”
		const float holdWindowHalfDeg = 6f;
		float curDeg = Mathf.Clamp(RodSegment1Hinge.jointAngle, RotRootMinAngleLimit, RotRootMaxAngleLimit);
		var limits = new JointAngleLimits2D
		{
			min = Mathf.Max(RotRootMinAngleLimit, curDeg - holdWindowHalfDeg),
			max = Mathf.Min(RotRootMaxAngleLimit, curDeg + holdWindowHalfDeg),
		};

		RodSegment1Hinge.useMotor = false;
		RodSegment1Hinge.limits = limits;   // 先数值
		RodSegment1Hinge.useLimits = true;     // 再启用
		hingeHoldActive = true;

		// 物理时间等待
		float duration = Mathf.Clamp(holdForwardDuration, 0.05f, 2.0f);
		int steps = Mathf.CeilToInt(duration / Time.fixedDeltaTime);

		for (int i = 0; i < steps; i++)
		{
			// 如果铰链对象没了，无法恢复，直接退出
			if (!RodSegment1Hinge) 
				yield break;

			// 外部取消（把 hingeHoldActive 设回 false）——立即恢复然后退出
			// 阶段改变（例如已不在 Flight）——恢复后提前结束（可按需改成继续等）
			if (!hingeHoldActive || currentPhase != Phase.Flight)
			{
				RestoreIfPossible();
				yield break;
			}
			yield return new WaitForFixedUpdate();
		}

		// 正常到时：恢复
		RestoreIfPossible();
		yield break;
	}

	// --------------------------- 视觉线 ---------------------------
	void ConfigureLineRenderer()
	{
		ropeMaxLength = Mathf.Max(ropeMaxLength, ropeMinLength + 0.01f);
		lineRendererSegments = Mathf.Clamp(lineRendererSegments, 2, 128);
		linePixelsPerUnit = Mathf.Max(16, linePixelsPerUnit);

		float w = 1f / Mathf.Max(1, linePixelsPerUnit);
		lineRenderer.widthMultiplier = 1f;
		lineRenderer.widthCurve = AnimationCurve.Constant(0f, 1f, w);
		lineRenderer.positionCount = Mathf.Max(2, lineRendererSegments) + 1;
	}

	void UpdateLineRenderer()
	{
		// 早退：组件未就绪
		if (!lineRenderer || !RodTipRB || !bobberRB) return;

		// 线段数量（N 段 = N+1 个点）
		int segmentCount = Mathf.Max(2, lineRendererSegments);
		if (lineRenderer.positionCount != segmentCount + 1)
			lineRenderer.positionCount = segmentCount + 1;

		// 两端点位置
		Vector3 tipWorldPos = RodTipRB.position;   // 竿梢刚体位置
		Vector3 bobberWorldPos = bobberRB.position;   // 浮标刚体位置

		// 端点间直线距离（仅用于曲线幅度估计）
		float straightDistance = Vector2.Distance(tipWorldPos, bobberWorldPos);

		// 为“下垂幅度”设置的距离上限（防止过长导致夸张）
		float sagDistanceClamped = Mathf.Min(straightDistance, ropeMaxLength * 1.5f);

		// 何时显示“下垂”（落水或收线阶段，且未气锁）
		bool shouldShowSagCurve =
			(currentPhase == Phase.Landed || currentPhase == Phase.Reeling) && !isAirLocked;

		// 下垂振幅比例（0~1），0 表示不下垂
		float sagAmplitude = shouldShowSagCurve ? 0.10f : 0f;

		// 逐段生成样条（简单正弦下垂）
		for (int i = 0; i <= segmentCount; i++)
		{
			float t01 = i / (float)segmentCount;                 // [0,1] 归一化插值参数
			Vector3 pos = Vector3.Lerp(tipWorldPos, bobberWorldPos, t01);
			pos.y -= sagAmplitude * sagDistanceClamped * Mathf.Sin(Mathf.PI * t01);
			lineRenderer.SetPosition(i, pos);
		}

		// 可视化调试：画端点直线
		if (debugDrawLineEnabled)
			Debug.DrawLine(tipWorldPos, bobberWorldPos, Color.magenta, 0f, false);

		// 空闲 / 蓄力阶段隐藏线
		bool shouldHideLine = (currentPhase == Phase.Idle || currentPhase == Phase.Charging);
		lineRenderer.enabled = !shouldHideLine;
	}

	// --------------------------- Safety ---------------------------
	void Watchdog()
	{
		if (currentPhase == Phase.Idle || !RodTipRB || !bobberRB) 
			return;
		float dist = Vector2.Distance(RodTipRB.position, bobberRB.position);
		if (dist > ropeMaxLength * 3f || float.IsNaN(dist)) // 极限保护
		{ 
			Debug.LogWarning($"[Fishing] Watchdog: dist={dist:F2} too large, RESET.", this);
			CancelInvoke();  ResetAll(true);
		}
	}

	// --------------------------- UI 渐隐 ---------------------------
	void StartUiFadeOut()
	{
		isUiFading = chargeSlider != null;
		uiFadeElapsed = 0f;
	}

	void TickUiFade(float dt)
	{
		if (!isUiFading || !chargeSlider) 
			return;

		const float dur = 0.35f;
		uiFadeElapsed += dt;
		float k = 1f - Mathf.Clamp01(uiFadeElapsed / dur);

		chargeSlider.value = k * chargeSlider.value;

		if (uiFadeElapsed >= dur)
		{
			chargeSlider.value = 0;
			chargeSlider.gameObject.SetActive(false);
			isUiFading = false;
		}
	}

	// --------------------------- Phase/Debug/HUD ---------------------------
	void ChangePhase(Phase next)
	{
		if (currentPhase == next)
			return;

		if (logPhaseChanges)
		{
			Debug.Log($"[Fishing] Phase: {currentPhase} -> {next}", this);
		}

		currentPhase = next;
		phaseMachine.Set(next);
	}

	// --------------------------- RodRoot 角度硬限制 ---------------------------
	void EnforceRodRootLimits()
	{
		if (!RodRootTransform)
			return;
		// 计算 RodRoot 当前相对 Home 的 Z 角度（-180..180）
		Quaternion rel = Quaternion.Inverse(RodRootHomeRotation) * RodRootTransform.localRotation;
		float zDegrees = NormalizeAngle180(rel.eulerAngles.z);
		float clamped = Mathf.Clamp(zDegrees, RotRootMinAngleLimit, RotRootMaxAngleLimit);
		if (!Mathf.Approximately(clamped, zDegrees))
		{
			RodRootTransform.localRotation = RodRootHomeRotation * Quaternion.Euler(0f, 0f, clamped);
		}
	}

	static float NormalizeAngle180(float deg)
	{
		deg %= 360f;
		if (deg > 180f) deg -= 360f;
		if (deg < -180f) deg += 360f;
		return deg;
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

	// --------------------------- 有限状态机（仅替换这一段） ---------------------------
	public class PhaseMachine
	{
		private Phase current;

		// 只承载“离散请求”的标志；由 Update 产出，FixedUpdate 消费
		private bool whipRequested;   // MouseUp 后请求进入甩竿（在 Fixed 中启动马达脉冲协程）
		private bool landedRequested;
		private bool reelRequested;   // 落水后点击开始收线

		public void Init(Phase initial)
		{
			current = initial;
			ClearRequests();
		}

		public void Set(Phase next)
		{
			current = next;
			// 进入任一新阶段后，旧请求一律清空，避免脏请求串场
			ClearRequests();
		}

		void ClearRequests()
		{
			whipRequested = false;
			landedRequested = false;
			reelRequested = false;
		}

		// ---- 生产者：只读输入 / 驱动非物理UI；不要在这里做物理 ----
		public void Tick(FishingPhysics2D context, float deltaTime)
		{
			switch (current)
			{
				case Phase.Idle:
					if (Input.GetMouseButtonDown(0))
					{
						context.TickIdle(); // 这里内部会 ChangePhase -> Charging（UI/外观 OK）
					}
					break;

				case Phase.Charging:
					context.TickCharging(deltaTime); // UI/角度过渡等视觉无物理
					if (Input.GetMouseButtonUp(0))
					{
						context.TickCharging(); // 计算 plannedRopeLength、UI 渐隐
						whipRequested = true;  // 仅发请求：在 Fixed 中启动马达脉冲协程
					}
					break;
				case Phase.Whip:
					context.TickWhip(deltaTime); //播放甩鱼竿动画。
					break;
				case Phase.Flight:
					if (Input.GetMouseButtonDown(0))
						landedRequested = true;
					break;

				case Phase.Landed:
					if (Input.GetMouseButtonDown(0))
						reelRequested = true; // 仅发请求：在 Fixed 中切到 Reeling
					break;

					// Whip / Flight / Reeling：连续物理阶段，不在 Update 做任何门控
			}
		}

		// ---- 消费者：所有物理都在这里按固定步执行 ----
		public void FixedTick(FishingPhysics2D context, float fixedDeltaTime)
		{
			switch (current)
			{
				case Phase.Charging:
					if (whipRequested)
					{
						whipRequested = false; // 立刻清零，避免重复起协程
						context.FixedTickCharging();
					}
					break;

				case Phase.Whip:
					// 连续物理：每个 FixedUpdate 都要推进甩竿计时与“是否到点释放”
					context.FixedTickWhip(fixedDeltaTime);
					break;

				case Phase.Flight:
					context.FixedTickFlight(fixedDeltaTime); // 里面已更新 cachedFlightBase

					int landingDecision = context.FixedTickCheckLanded(in context.cachedFlightBase);

					// 自动落地/空锁：立刻处理并退出本帧
					if (landingDecision == 1 || landingDecision == 2)
					{
						context.FixedTickCheckedLanded(landingDecision);
						return;
					}

					// “随时可收” 只在点击后触发
					if (landedRequested && landingDecision == 3)
					{
						landedRequested = false;
						context.FixedTickCheckedLanded(landingDecision);
						return;
					}
					break;
				case Phase.Landed:
					if (reelRequested)
					{
						reelRequested = false;
						context.FixedTickLanded(); // 转 Reeling（内部会 ChangePhase）
					}
					break;

				case Phase.Reeling:
					// 连续物理：每个 FixedUpdate 收线、加拉力、结束判定
					context.FixedTickReeling(fixedDeltaTime);
					break;
			}
		}
	}
}
