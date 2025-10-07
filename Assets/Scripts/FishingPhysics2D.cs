
// =========================================================
// FishingPhysics2D.cs — Unity 6 (6000.x) ）
// =========================================================

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using System;
using System.Collections;

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
	public float airDamping = 0.38f;
	public float waterDamping = 4.5f;

	[Header("Reel Tuning")]
	[Tooltip("distance 每秒缩短量")]
	public float reelDistancePerSecond = 6.5f;

	[Tooltip("沿绳方向额外拉力")]
	public float reelPullForce = 14f;

	[Header("Whip（甩竿）")]
	public float whipMotorAngularSpeed = 1000f;
	public float whipPulseDuration = 0.1f;

	[Header("Whip Animation（根节点动画）")]
	[Tooltip("Whip 阶段是否同时动画 RodRoot 角度")]
	public bool animateRodRootInWhipEnabled = true;

	[Tooltip("Whip 阶段的目标前倾角（度 0~80）")]
	[Range(0, 80)]
	public float maxForwardAngleDegreesInWhip = 60f;

	[Header("Launch Feel（手感增强）")]
	[Tooltip("蓄力曲线 >1 变硬，<1 变软")]
	public float powerCurveExponent = 1.20f;

	[Header("Launch Feel（竿身旋转）")]
	[Tooltip("蓄满时的最大后仰角（度 0~80）")]
	[Range(0, 80)]
	public float maxBackwardAngleDegrees = 60f;

	[Tooltip("Charging 时竿身旋转的平滑时间（秒）")]
	public float chargingRotationSmoothTime = 0.06f;

	[Header("Flight Lock（视觉友好落水）")]
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
	public bool landOnlyOnWaterEnabled = true;

	[Header("Water Landing（混合逻辑）")]
	[Tooltip("命中水后需要停留的最短时间，避免“刚碰水就落”")]
	public float waterLandingSettleDelay = 0.2f;

	[Tooltip("水中锁长的速度阈值，低于则可落")]
	public float waterLockSpeedThreshold = 0.8f;

	[Header("Rope Simple Control（精简线长控制）")]
	[Tooltip("最小绳长度")]
	public float ropeMinLength = 1.8f;

	[Tooltip("最大绳长度")]
	public float ropeMaxLength = 7.2f;

	[Tooltip("需要至少飞行的时间（秒）")]
	public float ropeMinFlightTime = 0.25f;

	[Tooltip("飞行超时时长（秒），超过则强制落水")]
	public float ropeFlightMaxTimeout = 1.2f;

	[Tooltip("解析前向飞行速度（m/s）")]
	public float ropeForwardFlightSpeed = 12f;

	[Tooltip("当前是否为无线长度模式（放开上限）")]
	public bool ropeUnlimitedEnabled = false;

	[Header("Rope Spool Out（飞行时放线）")]
	public bool ropeSpoolOutEnabled = true;

	[Tooltip("从最短放到计划长度所需时间（秒）")]
	public float ropeSpoolDuration = 0.28f;

	[Tooltip("放线时的额外松弛比例")]
	public float ropeSpoolSlackRatio = 0.06f;

	// === Charge → Speed Mapping（让蓄力影响初速度）===
	[Header("Charge → Speed Mapping")]
	[Tooltip("蓄力=0 时的基础前向速度（m/s）")]
	public float minForwardSpeed = 6f;
	[Tooltip("蓄力=1 时的基础前向速度（m/s）")]
	public float maxForwardSpeed = 19f;
	[Tooltip("蓄力到速度的指数，>1 硬，<1 软")]
	public float chargeToSpeedExponent = 1.20f;

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
	public bool snapRodOnLandEnabled = true;

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

	[Header("Charge Limit")]
	[Tooltip("把蓄力条映射为‘本次抛投的最大线长上限’")]
	public bool useChargeAsMax = true;

	[Header("Gravity Configs")]
	[Tooltip("飞行期的重力系数（顶视角 0.001~1）")]
	[Range(0.001f, 1f)]
	public float flightGravityScale = 0.18f;
	[Tooltip("放线期间使用的微重力（不影响放线逻辑）")]
	[Range(0.0001f, 0.01f)]
	public float spoolingMicroGravityScale = 0.01f;

	[Header("Flight Dynamics（飞行分段：起飞/匀速/降落）")]
	[Tooltip("起飞加速期的时长（秒），用正常重力 + 最大空气阻尼")]
	[Range(0.00f, 0.25f)]
	public float flightBoostDuration = 0.05f;

	[Tooltip("空中匀速期的线性阻尼（中等）")]
	[Range(0.00f, 5.0f)]
	public float cruiseLinearDamping = 0.25f;

	[Tooltip("起飞加速期的线性阻尼（最大）")]
	[Range(0.00f, 8.0f)]
	public float boostLinearDamping = 0.5f;

	[Tooltip("降落期的线性阻尼（最小）")]
	[Range(0.00f, 2.0f)]
	public float descentLinearDamping = 0.01f;

	[Tooltip("降落期重力系数的放大倍数（>1 会更快下坠、更有“砸水感”）")]
	[Range(0.5f, 3f)]
	public float descentGravityMultiplier = 1.35f;

	[Tooltip("进入匀速期的最低速度（m/s），起飞时速未达该值会继续视为“起飞加速”")]
	public float cruiseEnterSpeed = 5.5f;

	[Tooltip("进入降落期的向下速度阈值（m/s，纵向速度<-阈值）")]
	public float descentEnterDownSpeed = 2.0f;


	// 每次抛投时由蓄力计算出的“本次上限”
	float castMaxLength;

	float CurrentMaxCap =>
		ropeUnlimitedEnabled
			? ropeMaxLength
			: (useChargeAsMax ? Mathf.Clamp(castMaxLength, ropeMinLength, ropeMaxLength) : ropeMaxLength);

	[Header("Air Stop At Cap")]
	[Tooltip("飞行中一旦到达本次上限并已绷紧，立刻空中锁绳（关重力）")]
	public bool stopAtCapInAirEnabled = true;

	[Range(1, 4)]
	public int capTautFramesThreshold = 1; // 需要连续绷紧的帧数

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
	void FixedUpdate()
	{
		phaseMachine.FixedTick(this, Time.fixedDeltaTime);
	}

	void FixedTickWhip(float fixedDeltaTime)
	{
		Debug.Log($"[Fishing] WHIP chargeSilderValue={chargingSilderValue:F2} plannedRopeLength={plannedRopeLength:F2}", this);

		// —— 计时 & 把浮标继续“吸附在竿梢”（释放前都是整体）——
		whipElapsedTime += fixedDeltaTime;
		bobberRB.transform.localPosition = Vector3.zero;

		

		Vector2 targetReleaseDirection = (MouseSideSignOnScreen() > 0f) ? Vector2.left : Vector2.right;

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

		// —— 释放判定：到达“甩竿脉冲时长”后，等竿梢速度与抛掷方向至少有一定对齐度 —— //
		if (whipElapsedTime >= whipPulseDuration)
		{
			StartCoroutine(ReleaseBobberCoroutine(targetReleaseDirection));

			ChangePhase(Phase.Flight);
			if (events?.onRelease != null)
				events.onRelease.Invoke();

			// 竿前倾保持
			if (holdRodForwardEnabled)
				StartCoroutine(HoldRodForwardTemporarily());

			// 否则：再等一帧，让竿梢速度继续追上抛掷方向
			// 避免 dot≈0 的软脚，导致速度丢失进而导致方向有问题
		}
	}

	IEnumerator ReleaseBobberCoroutine(Vector2 targetReleaseDirection)
	{
		yield return new WaitForFixedUpdate();
		
		// 竿梢的世界“真实”速度（GetPointVelocity）（由 Seg3）
		Vector2 tipVelocityWorld = Vector2.zero;
		if (RodSegment3RB && RodTipRB)
		{
			tipVelocityWorld = RodSegment3RB
			? RodSegment3RB.GetPointVelocity(RodTipRB.position)
			: (RodTipRB ? RodTipRB.linearVelocity : Vector2.zero);
		}

		ReleaseBobberNow(targetReleaseDirection, tipVelocityWorld);

		flightSubphase = FlightSubphase.Boost;
		flightSubphaseLockedToDescent = false;
		bobberRB.gravityScale = flightGravityScale;
		bobberRB.linearDamping = Mathf.Max(boostLinearDamping, 0f);

		yield break;
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
		targetRopeLength = CurrentMaxCap;
		ropeSpoolElapsed = 0f;
	}

	// 只负责：把 bobber 打出去“够猛（前向） + 够抬（向上）”
	void ReleaseBobberNow(Vector2 directionHint, Vector2 rodTipVelocityWorld)
	{
		Debug.Log($"[Fishing] RELEASE rodTipVelocityWorld={rodTipVelocityWorld.magnitude:F2}", this);

		// ---------- 本函数内可调常量（命名清晰，无公开字段） ----------
		// 前向：基础底速 × 乘法增益 + 竿梢前向增益
		const float forwardBoostMultiplier = 1.18f;   // 前向乘法增益（>1 更猛）
		const float tipForwardGain = 1.30f;   // 竿梢前向贡献的放大系数
		const float backwardPreserveRatio = 0.10f;   // 竿梢若反向，保留的比例（避免“过零没劲”）

		// 向上：解析抛物线的竖直初速 v0y 叠加竿梢的上扬分量
		const float ballisticUpwardBlendRatio = 0.75f;   // 解析 v0y 与“竿梢上扬”的混合占比 0..1
		const float apexHeightAtFullChargeMeters = 2.8f;    // 满蓄力期望最高点（世界单位）
		const float minApexAtLowChargeMeters = 0.4f;    // 低蓄力时的最低最高点（防止太平）
		const float tipUpwardCarryRatio = 0.25f;   // 竿梢向上速度参与的系数（保留“抬头味道”）

		// 形状与安全：发射最小仰角 + 速度限幅（避免极端参数爆表）
		const float minimumLaunchAngleDegrees = 12f;     // 最小发射仰角（度）
		const float maximumLaunchSpeedMetersPerSecond = 40f;     // 软限幅（m/s）

		// ---------- 解绑整体，进入飞行的起步设定 ----------
		DetachFromTipKeepWorld();
		bobberRB.linearDamping = airDamping;
		bobberRB.gravityScale = flightGravityScale; // 单刚体重力比例，会叠乘 Physics2D.gravity。:contentReference[oaicite:0]{index=0}

		// ---------- 抛掷方向（永远有兜底） ----------
		Vector2 launchDirectionUnit =
			(directionHint.sqrMagnitude > 1e-12f)
				? directionHint.normalized
				: ((MouseSideSignOnScreen() > 0f) ? Vector2.left : Vector2.right);

		// ---------- 前向速度（猛） ----------
		// 1) 充能 → 基础底速（m/s）
		float charge01 = Mathf.Clamp01(chargingSilderValue);
		float baseForwardSpeedMetersPerSecond =
			Mathf.Lerp(minForwardSpeed, maxForwardSpeed,
				Mathf.Pow(charge01, Mathf.Max(1e-6f, chargeToSpeedExponent)));

		// 2) 额外“解析”前向底速（轻微加料，保持你原来的味道）
		baseForwardSpeedMetersPerSecond += ropeForwardFlightSpeed * 0.15f;

		// 3) 竿梢前向分量（只取在抛掷方向上的投影；若反向只保留一点点）
		float rodTipSpeedForwardMetersPerSecond = Vector2.Dot(rodTipVelocityWorld, launchDirectionUnit);
		float forwardFromRodTipMetersPerSecond =
			(rodTipSpeedForwardMetersPerSecond >= 0f)
				? rodTipSpeedForwardMetersPerSecond
				: rodTipSpeedForwardMetersPerSecond * backwardPreserveRatio;

		// 4) 汇总前向：底速 + 竿梢贡献 → 乘法增益
		float launchForwardSpeedMetersPerSecond =
			(baseForwardSpeedMetersPerSecond + forwardFromRodTipMetersPerSecond * tipForwardGain)
			* forwardBoostMultiplier;

		// 先得到“纯前向”的发射速度向量
		Vector2 launchVelocity = launchDirectionUnit * Mathf.Max(0f, launchForwardSpeedMetersPerSecond);

		// ---------- 向上分量（抬） ----------
		// 1) 有效重力 g（m/s^2）：|Physics2D.gravity.y| × 当前刚体 gravityScale
		float g = Mathf.Abs(Physics2D.gravity.y) * Mathf.Max(1e-6f, flightGravityScale);

		// 2) 目标最高点（随蓄力从 minApex 过渡到满蓄力 apex）
		float desiredApexMeters = Mathf.Lerp(minApexAtLowChargeMeters, apexHeightAtFullChargeMeters, Mathf.Pow(charge01, 0.85f));

		// 3) 解析竖直初速 v0y = sqrt(2 g h)
		float ballisticV0yMetersPerSecond = Mathf.Sqrt(Mathf.Max(0f, 2f * g * desiredApexMeters));

		// 4) 竿梢上扬分量（只取向上的一部分）与解析 v0y 混合
		float tipUpwardMetersPerSecond = Mathf.Max(0f, rodTipVelocityWorld.y) * tipUpwardCarryRatio;
		float addUpwardMetersPerSecond = Mathf.Lerp(
			tipUpwardMetersPerSecond,
			tipUpwardMetersPerSecond + ballisticV0yMetersPerSecond,
			Mathf.Clamp01(ballisticUpwardBlendRatio)
		);
		launchVelocity.y += addUpwardMetersPerSecond;

		// ---------- 最小仰角兜底（不改总速度幅值，只抬角度） ----------
		if (launchVelocity.magnitude > 1e-4f)
		{
			float minVy = launchVelocity.magnitude * Mathf.Sin(minimumLaunchAngleDegrees * Mathf.Deg2Rad);
			if (launchVelocity.y < minVy)
			{
				float newVy = minVy;
				float newVx = Mathf.Sign(launchVelocity.x) * Mathf.Sqrt(Mathf.Max(0f, launchVelocity.magnitude * launchVelocity.magnitude - newVy * newVy));
				launchVelocity = new Vector2(newVx, newVy);
			}
		}

		// ---------- 软限幅，防极端参数组合 ----------
		launchVelocity = Vector2.ClampMagnitude(launchVelocity, maximumLaunchSpeedMetersPerSecond);

		// 写回初速度（Unity 6 用 linearVelocity）
		bobberRB.linearVelocity = launchVelocity; // linearDamping 会随时间衰减线速度。

		// ---------- 放线：本次上限 = plannedRopeLength（仅限制“最大距离”） ----------
		castMaxLength = Mathf.Clamp(plannedRopeLength, ropeMinLength, ropeMaxLength);
		SetRopeForRelease(); // DistanceJoint2D.maxDistanceOnly=true ⇒ 只限制最大长度，类似悠悠球。

		// ---------- 清零统计 & 切阶段（起飞加速期：重力=飞行重力，阻尼=最大） ----------
		ropeFlightTimer = 0f; consecutiveTautFrames = 0; sawWater = false; waterHitTime = 0f;
	}

	public struct FlightingBaseData
	{
		public float cap;        // 本次上限（已考虑 charge/unlimited）
		public float tipTo;      // 竿梢->浮标 的距离
		public bool tight;       // 是否已绷紧
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
		data.tight = isTight;

		// 图层检测：水体（你已经有 waterLayerMask）
		data.inWaterNow = Physics2D.OverlapPoint(bobberRB.position, waterLayerMask) != null;

		return data;
	}

	// 其它阶段/放线/飞行/落水
	void FixedTickFlight(float fixedDeltaTime)
	{
		cachedFlightBase = BuildFlightingBaseData();

		// 放线插值推进（视觉平滑）
		if (ropeSpoolOutEnabled && bobberRopeJoint && bobberRopeJoint.maxDistanceOnly)
		{
			ropeSpoolElapsed += fixedDeltaTime;
			float newRopeLength = Mathf.Lerp(bobberRopeJoint.distance, targetRopeLength, Mathf.Clamp01(ropeSpoolElapsed / Mathf.Max(0.01f, ropeSpoolDuration)));
			bobberRopeJoint.distance = (newRopeLength > bobberRopeJoint.distance) ? newRopeLength : bobberRopeJoint.distance; // 防回退
		}

		ropeFlightTimer += fixedDeltaTime;
		if (!bobberRB || !bobberRopeJoint || !RodTipRB)
			return;

		// 离屏保护（只保留一套行为：空中锁）
		if (forceStopOffscreenEnabled && IsOffscreen(bobberRB.position, offscreenMarginRatio))
		{
			FixedTickCheckedLanded(1); // 等价“到cap空中锁”
			return;
		}

		// 水体命中仅做记录（放线阶段不触发落水）
		bool inWaterNow = Physics2D.OverlapPoint(bobberRB.position, waterLayerMask) != null;
		if (inWaterNow && !sawWater)
		{
			sawWater = true;
			waterHitTime = ropeFlightTimer;
		}

		// 基础量
		float cap = CurrentMaxCap;
		float tipTo = Vector2.Distance(RodTipRB.position, bobberRB.position);
		bool tight = tipTo >= bobberRopeJoint.distance - 0.01f;
		consecutiveTautFrames = tight ? (consecutiveTautFrames + 1) : 0;

		// 硬封顶：distance 不得超过 cap
		if (bobberRopeJoint.maxDistanceOnly)
			bobberRopeJoint.distance = Mathf.Min(bobberRopeJoint.distance, cap);

		// 兜底：即便关闭了 RopeSpoolOut，只要被拉紧也把 distance 往 cap 推
		if (!ropeSpoolOutEnabled && bobberRopeJoint.maxDistanceOnly && tight)
		{
			float slack = Mathf.Max(0f, ropeSpoolSlackRatio) * cap;
			bobberRopeJoint.distance = Mathf.Min(cap, Mathf.Max(bobberRopeJoint.distance, tipTo + slack));
		}

		// 三段式飞行：起飞加速 → 空中匀速 → 降落加速
		bool spoolingHard = bobberRopeJoint.maxDistanceOnly && bobberRopeJoint.distance < cap - 0.005f;

		// —— 子阶段切换条件 —— //
		if (!flightSubphaseLockedToDescent)
		{
			// 起飞 → 匀速：满足最低飞行时间，速度达到阈值
			if (flightSubphase == FlightSubphase.Boost)
			{
				bool boostTimeOk = ropeFlightTimer >= Mathf.Max(0.01f, flightBoostDuration);
				bool speedOk = bobberRB.linearVelocity.magnitude >= Mathf.Max(0f, cruiseEnterSpeed);
				if (boostTimeOk && speedOk)
					flightSubphase = FlightSubphase.Cruise;
			}

			// 匀速 → 降落：出现较明显“向下速度”或接近 cap 或刚入水后稳定
			if (flightSubphase == FlightSubphase.Cruise)
			{
				bool goingDownFast = bobberRB.linearVelocity.y <= -Mathf.Abs(descentEnterDownSpeed);
				bool waterSettle = sawWater && (ropeFlightTimer - waterHitTime) >= Mathf.Max(0f, waterLandingSettleDelay) * 0.5f;

				if (goingDownFast || (tipTo >= cap - capTolerance) || waterSettle)
				{
					flightSubphase = FlightSubphase.Descent;
					flightSubphaseLockedToDescent = true;
				}
			}
		}

		// —— 根据子阶段设置重力/阻尼（并在“放线中”对重力做微调）—— //
		switch (flightSubphase)
		{
			case FlightSubphase.Boost:
				// 起飞：正常飞行重力 + 最大阻尼，前段弧线立刻可见
				bobberRB.gravityScale = flightGravityScale;
				bobberRB.linearDamping = Mathf.Max(boostLinearDamping, 0f);
				// 若仍在“硬放线”，前 2~3 帧允许直接 return，保持你原逻辑的“起弧”
				if (ropeFlightTimer < Mathf.Max(0.02f, Mathf.Min(0.06f, flightBoostDuration)))
					return;
				break;

			case FlightSubphase.Cruise:
				// 匀速：微重力 + 中等阻尼
				if (spoolingHard)
				{
					// 放线阶段：把重力从 flight → micro 平滑插值（速度越快越接近 micro）
					float v = bobberRB.linearVelocity.magnitude;
					float vHi = Mathf.Max(0.10f, ropeForwardFlightSpeed * 0.90f);
					float vLo = Mathf.Max(0.05f, ropeForwardFlightSpeed * 0.30f);
					float tSmooth = Smooth01C3(Mathf.Clamp01(Mathf.InverseLerp(vLo, vHi, v)));
					float g = Mathf.Lerp(flightGravityScale, Mathf.Max(0f, spoolingMicroGravityScale), tSmooth);
					bobberRB.gravityScale = g;
					// 仍跳过落水
					bobberRB.linearDamping = Mathf.Max(cruiseLinearDamping, 0f);
				}
				else
				{
					// 放线结束：常态微重力
					bobberRB.gravityScale = Mathf.Max(0f, spoolingMicroGravityScale);
					bobberRB.linearDamping = Mathf.Max(cruiseLinearDamping, 0f);
				}
				break;

			case FlightSubphase.Descent:
				// 降落：重力加强（加速向下改变姿态）+ 最小阻尼（不“拖死”）
				bobberRB.gravityScale = flightGravityScale * Mathf.Max(0.01f, descentGravityMultiplier);
				bobberRB.linearDamping = Mathf.Max(descentLinearDamping, 0f);
				break;
		}

		// 放在 FixedTickFlight 的结尾做个轻量日志（必要时再关）
		if ((int)(ropeFlightTimer * 60) % 6 == 0) // 每6帧采样一次
		{
			Debug.LogWarning($"[Flight] sub={flightSubphase} gScale={bobberRB.gravityScale:F3} " +
					  $"v=({bobberRB.linearVelocity.x:F2},{bobberRB.linearVelocity.y:F2}) " +
					  $"dist={Vector2.Distance(RodTipRB.position, bobberRB.position):F2} " +
					  $"rope={bobberRopeJoint.distance:F2}/{CurrentMaxCap:F2}");
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
		if (stopAtCapInAirEnabled && (cacheData.tipTo >= cacheData.cap - Mathf.Abs(capTolerance)) && ropeFlightTimer >= ropeMinFlightTime)
			return 1;

		// 非水早落水（允许时）：绷紧达到阈值且飞行时间达标
		if (!landOnlyOnWaterEnabled && consecutiveTautFrames >= Mathf.Max(1, tautFramesThresholdToLand) && ropeFlightTimer >= ropeMinFlightTime)
			return 2;

		// 水面稳定落水：仍然需要 inward/slowInWater（它们只在这里用，算一次即可）
		bool inward = Vector2.Dot(RodTipRB.position - bobberRB.position, bobberRB.linearVelocity) > 0f || bobberRB.linearVelocity.sqrMagnitude < 0.0003f;
		bool slowInWater = bobberRB.linearVelocity.sqrMagnitude <= waterLockSpeedThreshold * waterLockSpeedThreshold;

		if (ShouldForceLand(cacheData.inWaterNow, cacheData.tight, inward, slowInWater))
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
		float cur = Vector2.Distance(RodTipRB.position, bobberRB.position);
		// 落水后的实际长度 ∈ [min, 本次上限]
		float clamped = Mathf.Clamp(cur, ropeMinLength, CurrentMaxCap);

		Debug.Log($"[Fishing] LANDED raw={cur:F2} clamp={clamped:F2} cap={CurrentMaxCap:F2}", this);

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
