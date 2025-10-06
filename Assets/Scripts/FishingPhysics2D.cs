
// =========================================================
// FishingPhysics2D.cs — Unity 6 (6000.x) ）
// =========================================================

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using System;

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
	public float whipMotorAngularSpeed = 1100f;
	public float whipPulseDuration = 0.14f;

	[Header("Whip Animation（根节点动画）")]
	[Tooltip("Whip 阶段是否同时动画 RodRoot 角度")]
	public bool animateRodRootInWhipEnabled = true;

	[Tooltip("Whip 阶段的目标前倾角（度 0~80）")]
	[Range(0, 80)]
	public float maxForwardAngleDegreesInWhip = 60f;

	[Header("Launch Feel（手感增强）")]
	[Tooltip("竿梢速度增益")]
	public float tipVelocityMultiplier = 1.30f;

	[Tooltip("解析额外速度增益")]
	public float extraVelocityMultiplier = 1.00f;

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

	float targetRopeLength;
	float ropeSpoolElapsed;

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

	[Header("Gravity Configs")]
	[Tooltip("飞行期的重力系数（顶视角 0.10~0.30）")]
	[Range(0.01f, 9.81f)]
	public float flightGravityScale = 0.18f;
	[Tooltip("放线期间使用的微重力（不影响放线逻辑）")]
	[Range(0.0001f, 0.01f)]
	public float spoolingMicroGravityScale = 0.01f;

	[Header("Micro/Cap Lock")]
	[Tooltip("到达cap后是否把浮标设为Kinematic（完全免疫力）")]
	public bool setBobberKinematicAtCap = true;

	[Header("Charge Limit")]
	[Tooltip("把蓄力条映射为‘本次抛投的最大线长上限’")]
	public bool useChargeAsMax = true;

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
	public float capTolerance = 0.02f;

	[Tooltip("离屏时是否直接空中锁死")]
	public bool forceStopOffscreenEnabled = true;

	// ------------------------- 内部状态 -------------------------
	public enum Phase { Idle, Charging, Whip, Flight, Landed, Reeling }
	Phase currentPhase = Phase.Idle;
	PhaseMachine phaseMachine = new PhaseMachine();

	LineRenderer lineRenderer;
	float castTimer, plannedRopeLength;
	float uiFadeElapsed; bool isUiFading;
	float whipElapsedTime;           // 甩竿计时
	int consecutiveTautFrames;                      // 连续“已绷紧”计数
	float chargingSilderValue;
	bool isAirLocked; // 空中锁定后免疫一切外力

	// 竿梢“真实”速度（由 Seg3 刚体在 FixedUpdate 计算）
	Vector2 tipVelocityFixed;

	// 鼠标决定的当前鱼竿顺时针逆时针方向
	private Vector2 ChargeDirection = Vector2.zero;
	// 鼠标决定的当前鱼竿浮标抛投方向（相对竿梢）
	private Vector2 ReleaseWhipDirection = Vector2.zero;

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
	float maxCastDistance;  // 历史最远落水长度
							// 水命中计时
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

	// 物理步里**直接**用刚体求竿梢速度（旋转→切向速度）
	void FixedUpdate()
	{
		if (RodSegment3RB && RodTipRB)
		{
			// 关键：用动态段 RigidBody2D 的 GetPointVelocity 取世界上一点的线速度
			// 即便 RodTip 本身是 Kinematic 也能得到真实速度
			tipVelocityFixed = RodSegment3RB.GetPointVelocity(RodTipRB.position);
		}
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
	// ② 参考右向：相机的 right 投到 XY
	Vector2 ScreenRight2D()
	{
		if (Camera.main == null) 
			return Vector2.right;
		Vector3 r3 = Camera.main.transform.right;              // Transform.right（世界红轴） 
		return new Vector2(r3.x, r3.y).normalized;              // 投到XY平面
	}

	// ③ 鼠标世界坐标（2D 正交：给 z=0；透视：给到“竿梢所在平面”的深度）
	Vector2 MouseWorld2D()
	{
		if (Camera.main == null) 
			return Vector2.zero;
		var mp = currentMousePosition;

		if (Camera.main.orthographic)
		{
			return (Vector2)Camera.main.ScreenToWorldPoint(new Vector3(mp.x, mp.y, 0f));
		}
		else
		{
			float tipZ = (RodTipRB != null) ? RodTipRB.transform.position.z : transform.position.z;
			float depthFromCam = Mathf.Abs(Camera.main.transform.position.z - tipZ);
			return (Vector2)Camera.main.ScreenToWorldPoint(new Vector3(mp.x, mp.y, depthFromCam));
		}
	}

	// 以“屏幕右”为参考，判断鼠标相对 RodRoot (鱼竿) 在左(+1)还是右(-1)
	float MouseSideSignOnScreen()
	{
		Vector2 referenceRight = ScreenRight2D();
		Vector2 pivot = RodRootTransform ? (Vector2)RodRootTransform.position : (Vector2)transform.position;
		Vector2 toMouse = MouseWorld2D() - pivot;
		if (toMouse.sqrMagnitude < 1e-12f) 
			return +1f; // 鼠标几乎在枢轴上：默认左(+)

		toMouse.Normalize();
		// [-180,180] 的带符号最小夹角
		// 左(+)/右(-)
		return (Vector2.SignedAngle(referenceRight, toMouse) >= 0f) ? +1f : -1f;                     
	}

	// ④  以“屏幕右”为参考 判断鼠标相对 RodRoot (鱼竿) 取“后仰”的侧向（在左(+1)还是右(-1) 不使用 Sign；用带符号夹角 ）
	Vector2 GetChargeDirectionFromScreen()
	{
		// 参考“屏幕右”方向（相机 Transform.right 投到 XY）
		Vector2 referenceRight = ScreenRight2D();

		Camera mainCamera = Camera.main;
		if (!mainCamera)
			return referenceRight;

		// 用“杆根”当基点，更符合“转杆”的语义
		Vector2 pivot = RodRootTransform ? (Vector2)RodRootTransform.position
										 : (Vector2)transform.position;

		// 2D 正交：z=0；若换透视，用到 pivot 的深度
		float depth = mainCamera.orthographic
			? 0f
			: Mathf.Abs(mainCamera.transform.position.z - (RodRootTransform ? RodRootTransform.position.z : transform.position.z));

		Vector2 mouse = (Vector2)mainCamera.ScreenToWorldPoint(new Vector3(currentMousePosition.x, currentMousePosition.y, depth));
		Vector2 toMouse = mouse - pivot;

		// 鼠标几乎在枢轴上：退回参考右
		if (toMouse.sqrMagnitude < 1e-12f)
			return referenceRight;

		// 用 if 判断左右半平面
		float angle = Vector2.SignedAngle(referenceRight, toMouse);
		if (angle >= 0f)
			return referenceRight;
		else
			return -referenceRight;
	}

	// ⑤ 前甩(前倾)方向=与后仰相反
	Vector2 GetReleaseWhipDirectionFromScreen()
	{
		return -GetChargeDirectionFromScreen();
	}

	// --------------------------- 状态实现 ---------------------------
	void BeginCharging()
	{
		ChangePhase(Phase.Charging);
		castTimer = 0f;

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

		// 明确计算“真正的旋转方向”（与鼠标水平相同）
		ChargeDirection = GetChargeDirectionFromScreen();

		// 规则：点左边 => 竿向左后仰；点右边 => 竿向右后仰
		// 甩竿时根节的“后仰”动画（仅做外观过渡）
		float targetAngle = MouseSideSignOnScreen() * Mathf.Abs(maxBackwardAngleDegrees) * ChargeAngleRatio;
		// 可选：如果发现精灵朝向和数学正负相反，把下面这一行解注释
		// targetDegrees = -targetDegrees;

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
		plannedRopeLength = Mathf.Lerp(ropeMinLength, ropeMaxLength, planned01); // 1% 就是 1%

		if (chargeSlider)
			StartUiFadeOut();

		ChangePhase(Phase.Whip);

		if (events?.onWhipBegin != null)
			events.onWhipBegin.Invoke();

		Debug.Log("[Fishing] Whip start", this);
	}

	void TickWhip(float deltaTime)
	{
		// 明确计算“真正的抛出方向”（与鼠标水平相反）
		ReleaseWhipDirection = GetReleaseWhipDirectionFromScreen();

		StartPulseHingeMotorTowardMouse();

		Debug.Log($"[Fishing] WHIP chargeSilderValue={chargingSilderValue:F2} plannedRopeLength={plannedRopeLength:F2}", this);

		// —— 计时 & 把浮标继续“吸附在竿梢”（释放前都是整体）——
		whipElapsedTime += deltaTime;
		bobberRB.transform.localPosition = Vector3.zero;

		// 竿梢的世界速度（FixedUpdate 里用 GetPointVelocity 算好的）
		Vector2 tipVelocityWorld = tipVelocityFixed;

		// —— 甩竿时根节的“后仰 → 前倾”动画（仅做外观过渡）——
		if (animateRodRootInWhipEnabled && RodRootTransform)
		{
			float whipProgress01 = Mathf.Clamp01(whipElapsedTime / Mathf.Max(0.02f, whipPulseDuration)); // 0..1 进度
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
			// 满足“超时”，就释放；方向保持“反侧”的 castDirUnit
			if (whipElapsedTime >= whipPulseDuration)
			{
				ReleaseNow(ReleaseWhipDirection, tipVelocityWorld);
			}
			// 否则：再等一帧，让竿梢速度继续追上抛掷方向
			// 避免 dot≈0 的软脚，导致速度丢失进而导致方向有问题
		}
	}

	//在 Whip 开始时，给第一节鱼竿关节来一脚“马达脉冲”，把竿子朝将要甩线的方向迅速带一下，让甩竿更有力
	void StartPulseHingeMotorTowardMouse()
	{
		if (!RodSegment1Hinge || !RodSegment1RB || !RodTipRB)
			return;

		// 当前轴向：第一节枢轴 -> 竿梢（单位向量）
		Vector2 currentAxisDirection = ((Vector2)RodTipRB.position - (Vector2)RodSegment1RB.position).normalized;

		// 目标轴向：已算好的“甩线方向”（与鼠标相反侧）；退化兜底为右
		Vector2 targetAxisDirection = (ReleaseWhipDirection.sqrMagnitude > 1e-12f)
			? ReleaseWhipDirection.normalized
			: Vector2.right;

		// 角距离（幅值）：由两单位向量的弦长得到 θ = 2 * asin(|u - v| / 2)
		float chordLength = (currentAxisDirection - targetAxisDirection).magnitude;          // ∈ [0, 2]
		float angleRadians = 2f * Mathf.Asin(Mathf.Clamp01(chordLength * 0.5f));            // ∈ [0, π]
		float angleDegrees = angleRadians * Mathf.Rad2Deg;                                   // 角度幅值（不带符号）

		// 方向：用 DeltaAngle 求“最短路带符号角差”（-180°..+180°）
		float currentAxisAngleDeg = Mathf.Atan2(currentAxisDirection.y, currentAxisDirection.x) * Mathf.Rad2Deg;
		float targetAxisAngleDeg = Mathf.Atan2(targetAxisDirection.y, targetAxisDirection.x) * Mathf.Rad2Deg;
		float signedShortestDeltaDegrees = Mathf.DeltaAngle(currentAxisAngleDeg, targetAxisAngleDeg); // 可能为负

		// 目标角速度（度/秒）： 用“角差/脉冲时长”作为理想速度，再夹到你的最大马达速度
		float idealSpeedDegPerSec = angleDegrees / Mathf.Max(0.02f, whipPulseDuration);
		float maxAbsDegPerSec = Mathf.Abs(whipMotorAngularSpeed);
		// 不用 Sign：直接用 delta 自带的正负号
		float motorSpeedDegreesPerSec = Mathf.Clamp(
			signedShortestDeltaDegrees / Mathf.Max(0.02f, whipPulseDuration),
			-maxAbsDegPerSec, +maxAbsDegPerSec
		);

		// 如果角差极小，就不必抽一下马达（可选）
		if (Mathf.Abs(signedShortestDeltaDegrees) < 0.01f)
			return;

		// 下发马达设置（注意 JointMotor2D 是 struct，改完要回写回去才生效）
		var motor = RodSegment1Hinge.motor;
		motor.motorSpeed = motorSpeedDegreesPerSec; // 正=逆时针，负=顺时针（单位：度/秒）
		motor.maxMotorTorque = 1e6f;                    // 给足扭矩，确保能带得动
		RodSegment1Hinge.motor = motor;              // 关键：回写
		RodSegment1Hinge.useMotor = true;

		// 过一个“脉冲时长”后关掉马达（避免持续驱动）
		CancelInvoke(nameof(StopHingeMotor));
		Invoke(nameof(StopHingeMotor), Mathf.Clamp(whipPulseDuration * 1.1f, 0.01f, 0.2f));
	}

	void StopHingeMotor()
	{
		if (RodSegment1Hinge)
			RodSegment1Hinge.useMotor = false;
	}

	void ReleaseNow(Vector2 direction, Vector2 tipVelocityWorld)
	{
		// —— 基础前向速度（解析项），保留你原来的手感参数 —— //
		// 释义：哪怕竿梢没给力，也保证有个“往前飞”的底速
		float baseForwardSpeed = ropeForwardFlightSpeed * Mathf.Max(0f, extraVelocityMultiplier);

		// —— 从“整体状态”解绑，进入飞行 —— //
		DetachFromTipKeepWorld();
		bobberRB.linearDamping = airDamping;
		bobberRB.gravityScale = flightGravityScale;  // 抛物线感觉（顶视角/2D 也需要重力视觉） 

		// —— 方向：只用“单位方向”，不改方向，只算力度 —— //
		Vector2 useDirection = (direction.sqrMagnitude > 1e-12f) ? direction.normalized : Vector2.right;

		// —— 把竿梢速度分解到“前向/侧向”，并做一点点折算 —— //
		// tipForwardDot：竿梢速度在抛掷方向上的投影（点积）
		// tipSpeedSqrt  ：竿梢总速模平方
		float tipForwardDot = Vector2.Dot(tipVelocityWorld, useDirection);
		float tipSpeedSqrt = tipVelocityWorld.sqrMagnitude;

		// 侧向分量模：|v|^2 - dot^2（数值更稳；<0 夹 0 防止浮点误差）
		float sideSpeedSq = Mathf.Max(0f, tipSpeedSqrt - tipForwardDot * tipForwardDot);
		float sideSpeed = Mathf.Sqrt(sideSpeedSq);

		// —— 手感系数（你可以在 Inspector 抽出去） —— //
		// 给“一点反向”也少许力量（避免刚过零就‘没劲’）；侧向折一部分到前进方向（甩竿更有冲）
		const float keepBackwardRatio = 0.15f;  // 0~0.5：反向保留几成
		const float sideToForwardMix = 0.35f;  // 0~1  ：侧向折到前向的比例

		// 如果竿梢前向分量是反的，就只保留一小部分；否则全保留
		float forwardFromTip = (tipForwardDot >= 0f) ? tipForwardDot : tipForwardDot * keepBackwardRatio;

		// 侧向折算到前进
		float forwardFromSide = sideSpeed * sideToForwardMix;

		// 合成“来自竿梢”的有效前向力度，再叠加解析前向底速
		float effectiveTipForward = forwardFromTip + forwardFromSide;

		// 竿梢增益：给竿梢贡献再放大一点（你原来的 tipVelocityMultiplier）
		float tipGain = Mathf.Max(0f, tipVelocityMultiplier);

		// 最终初速标量（沿 castDir）
		float initialSpeedMag = effectiveTipForward * tipGain + baseForwardSpeed;

		// 初速度向量（只沿 castDir）
		Vector2 initialVelocity = useDirection * Mathf.Max(0f, initialSpeedMag);

		// 给一点“抬杆上扬”：取竿梢世界速度里的“向上分量”的一小部分
		initialVelocity.y += Mathf.Max(0f, tipVelocityWorld.y) * 0.25f; // 0.25f 可改成 0.2~0.35f

		bobberRB.linearVelocity = initialVelocity;

		// —— 线控：释放时设置（本次上限=plannedRopeLength，飞行期只限制“最大距离”）—— //
		castMaxLength = Mathf.Clamp(plannedRopeLength, ropeMinLength, ropeMaxLength);
		// 这里会开启 DistanceJoint2D.maxDistanceOnly=true 的模式
		SetRopeForRelease();

		// —— 切换阶段/清零统计 —— //
		ChangePhase(Phase.Flight);
		castTimer = 0f;
		consecutiveTautFrames = 0;
		sawWater = false;
		waterHitTime = 0f;

		if (events?.onRelease != null)
			events.onRelease.Invoke();

		Debug.Log($"[Fishing] RELEASE vTip={tipVelocityWorld.magnitude:F2} v0={baseForwardSpeed:F2}", this);

		// 抛出后短暂保持前倾（可选）
		if (holdRodForwardEnabled)
			HoldRodForwardTemporarily();
	}

	void TickFlight(float deltaTome)
	{
		// 1) 放线插值推进（视觉平滑）
		if (ropeSpoolOutEnabled && bobberRopeJoint && bobberRopeJoint.maxDistanceOnly)
		{
			ropeSpoolElapsed += deltaTome;
			float newRopeLength = Mathf.Lerp(bobberRopeJoint.distance, targetRopeLength, Mathf.Clamp01(ropeSpoolElapsed / Mathf.Max(0.01f, ropeSpoolDuration)));
			bobberRopeJoint.distance = (newRopeLength > bobberRopeJoint.distance) ? newRopeLength : bobberRopeJoint.distance; // 防回退
		}

		castTimer += deltaTome;
		if (!bobberRB || !bobberRopeJoint || !RodTipRB)
			return;

		// 2) 离屏保护（只保留一套行为：空中锁）
		if (forceStopOffscreenEnabled && IsOffscreen(bobberRB.position, offscreenMarginRatio))
		{
			AirLockAtCap();
			return;
		}

		// 3) 水体命中仅做记录（放线阶段不触发落水）
		bool inWaterNow = Physics2D.OverlapPoint(bobberRB.position, waterLayerMask) != null;
		if (inWaterNow && !sawWater)
		{
			sawWater = true;
			waterHitTime = castTimer;
		}

		// 4) 基础量
		float cap = CurrentMaxCap;
		float tipTo = Vector2.Distance(RodTipRB.position, bobberRB.position);
		bool tight = tipTo >= bobberRopeJoint.distance - 0.01f;
		consecutiveTautFrames = tight ? (consecutiveTautFrames + 1) : 0;

		// 5) 硬封顶：distance 不得超过 cap
		if (bobberRopeJoint.maxDistanceOnly)
			bobberRopeJoint.distance = Mathf.Min(bobberRopeJoint.distance, cap);

		// 6) 兜底：即便关闭了 RopeSpoolOut，只要被拉紧也把 distance 往 cap 推
		if (!ropeSpoolOutEnabled && bobberRopeJoint.maxDistanceOnly && tight)
		{
			float slack = Mathf.Max(0f, ropeSpoolSlackRatio) * cap;
			bobberRopeJoint.distance = Mathf.Min(cap, Mathf.Max(bobberRopeJoint.distance, tipTo + slack));
		}

		// 7) 放线进行中：按速度在“微重力↔飞行重力”之间平滑过渡
		bool spoolingHard = bobberRopeJoint.maxDistanceOnly && bobberRopeJoint.distance < cap - 0.005f;
		if (spoolingHard)
		{
			// 起飞的前几帧：先用正常重力，保证一上来就有弧线
			if (castTimer < 0.06f) // 2~3 帧即可
			{
				bobberRB.gravityScale = flightGravityScale;
				return;
			}

			// 基于速度做重力比例：v >= vHi -> 更接近 micro；v <= vLo -> 更接近 flight
			// 不新增公开字段，这里用 ropeForwardFlightSpeed 推出两个本地阈值
			float v = bobberRB.linearVelocity.magnitude;
			const float vHiRatio = 0.90f;  // 高速阈值：越接近 1 越苛刻
			const float vLoRatio = 0.60f;  // 低速阈值：越接近 0 越宽松
			float vHi = Mathf.Max(0.10f, ropeForwardFlightSpeed * vHiRatio);
			float vLo = Mathf.Max(0.05f, ropeForwardFlightSpeed * vLoRatio);

			// t=0 用 flightGravity，t=1 用 microGravity；用你已有的 Smooth01C3 做顺滑
			float tLinear = Mathf.InverseLerp(vLo, vHi, v);     // 0..1（速度越快越接近 1）
			float tSmooth = Smooth01C3(Mathf.Clamp01(tLinear)); // 平滑曲线，避免重力突变

			float g = Mathf.Lerp(flightGravityScale, Mathf.Max(0f, spoolingMicroGravityScale), tSmooth);
			bobberRB.gravityScale = g;

			// 放线期仍跳过落水逻辑
			return;
		}

		// 放线结束：恢复飞行重力（若未锁）
		if (!isAirLocked)
			bobberRB.gravityScale = flightGravityScale;

		// 8) 到达 cap：以真实距离判定并空中锁
		bool nearCap = tipTo >= cap - capTolerance;
		if (stopAtCapInAirEnabled && nearCap && castTimer >= ropeMinFlightTime)
		{
			AirLockAtCap();
			return;
		}

		// 9) 允许“非水面早落水”才走这一支
		if (!landOnlyOnWaterEnabled && consecutiveTautFrames >= Mathf.Max(1, tautFramesThresholdToLand) && castTimer >= ropeMinFlightTime)
		{
			LockRopeAtCurrentAndWaterDamping(); // 老的“落入水/地”锁法
			return;
		}

		// 10) 水面落水（稳定后判定）
		Vector2 toTip = RodTipRB.position - bobberRB.position;
		bool inward = Vector2.Dot(toTip, bobberRB.linearVelocity) > 0f
					|| bobberRB.linearVelocity.sqrMagnitude < 0.0004f;
		bool slowInWater = bobberRB.linearVelocity.sqrMagnitude <= waterLockSpeedThreshold * waterLockSpeedThreshold;
		if (ShouldForceLand(inWaterNow, tight, inward, slowInWater))
		{
			LockRopeAtCurrentAndWaterDamping();
			return;
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

		float cap = CurrentMaxCap;
		float tipTo = Vector2.Distance(RodTipRB.position, bobberRB.position);
		float clamped = Mathf.Clamp(tipTo, ropeMinLength, cap);

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
		ChangePhase(Phase.Landed); // 此时左键可开始收线
		Debug.Log($"[Fishing] AIR-LOCK cap={cap:F2} tipTo={tipTo:F2}", this);
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

		ChangePhase(Phase.Landed);

		maxCastDistance = Mathf.Max(maxCastDistance, cur);

		if (events?.onLand != null)
			events.onLand.Invoke();

		if (snapRodOnLandEnabled)
			RestoreRodPoseOnly();

		Debug.Log($"[Fishing] LANDED lockLen={cur:F2}", this);
	}

	void TryBeginReelFromFlight()
	{
		if (!Input.GetMouseButtonDown(0) || currentPhase != Phase.Flight || bobberRopeJoint == null || RodTipRB == null)
			return;

		if (canReelAnytime)
		{
			// 直接强制落水并开始收线
			LockRopeAtCurrentAndWaterDamping();
			BeginReel();
			Debug.Log("[Fishing] Begin reel from Flight (anytime)", this);
			return;
		}

		// 原有逻辑：超时且已绷紧
		if (castTimer < ropeMinFlightTime) 
			return;

		if (Vector2.Distance(RodTipRB.position, bobberRB.position) < 0.01f) 
			return; // 未绷紧

		LockRopeAtCurrentAndWaterDamping();
		BeginReel();
		Debug.Log("[Fishing] Begin reel from Flight (tight & timeout)", this);
	}

	void BeginReel()
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

		ChangePhase(Phase.Reeling);
		if (bobberRopeJoint) bobberRopeJoint.maxDistanceOnly = false; // 绝对长度模式：distance 变短会“拖回来”
		if (events?.onReelStart != null) events.onReelStart.Invoke();
		Debug.Log("[Fishing] Reel start", this);
	}

	// 统一设置：释放时线状态（使用精简参数）
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

	bool ShouldForceLand(bool inWaterNow, bool tight, bool inward, bool slowInWater)
	{
		bool timeout = castTimer >= ropeFlightMaxTimeout;

		if (landOnlyOnWaterEnabled)
		{
			if (!inWaterNow)
				return false;

			bool settleOk = sawWater && (castTimer - waterHitTime) >= Mathf.Max(0f, waterLandingSettleDelay);
			// 只在“绷紧或超时”并且水面稳定后才落水
			return settleOk && (tight || timeout);
		}
		else
		{
			return (tight && castTimer >= ropeMinFlightTime) || timeout;
		}
	}

	void TickReeling(float dt)
	{
		if (bobberRopeJoint)
			bobberRopeJoint.distance = Mathf.Max(0f, bobberRopeJoint.distance - Mathf.Max(0f, reelDistancePerSecond) * dt);

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

		hingeHoldActive = false; // 防止误判

		// —— 关闭所有关节的 motor/limits —— //
		ResetHinge(RodSegment1Hinge);
		ResetHinge(RodSegment2Hinge);
		ResetHinge(RodSegment3Hinge);

		// —— 基本标量清零 —— //
		castTimer = 0f;
		plannedRopeLength = 0f;

		isUiFading = false;
		uiFadeElapsed = 0f;

		whipElapsedTime = 0f;
		consecutiveTautFrames = 0;

		tipVelocityFixed = Vector2.zero;

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

	// --------------------------- 竿前倾保持 ---------------------------
	void HoldRodForwardTemporarily()
	{
		if (!RodSegment1Hinge || hingeHoldActive) 
			return;

		// 记录当前设置
		hingePrevUseLimits = RodSegment1Hinge.useLimits;
		hingePrevUseMotor = RodSegment1Hinge.useMotor;
		hingePrevLimits = RodSegment1Hinge.limits;
		hingePrevMotor = RodSegment1Hinge.motor;

		// 以释放瞬间角度为中心，设置窄窗口的角度限制
		float baseAngle = Mathf.Clamp(RodSegment1Hinge.jointAngle, RotRootMinAngleLimit, RotRootMaxAngleLimit); // 当前角度（度）
		var limits = new JointAngleLimits2D
		{
			min = RotRootMinAngleLimit,
			max = RotRootMaxAngleLimit
		};

		RodSegment1Hinge.useMotor = false; // 停止进一步驱动，避免与限制冲突
		RodSegment1Hinge.limits = limits; // 必须先设置 limits 再启用 useLimits
		RodSegment1Hinge.useLimits = true;

		hingeHoldActive = true;
		Invoke(nameof(ReleaseHoldRodForward), Mathf.Clamp(holdForwardDuration, 0.05f, 2.0f));
	}

	void ReleaseHoldRodForward()
	{
		if (!RodSegment1Hinge || !hingeHoldActive) 
			return;

		// 恢复原始设置
		RodSegment1Hinge.useLimits = hingePrevUseLimits;
		RodSegment1Hinge.limits = hingePrevLimits;
		RodSegment1Hinge.useMotor = hingePrevUseMotor;
		RodSegment1Hinge.motor = hingePrevMotor;

		hingeHoldActive = false;

		// 保险：确保完全关闭
		ResetHinge(RodSegment1Hinge);
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

	// --------------------------- 有限状态机（同文件内隔离） ---------------------------
	public class PhaseMachine
	{
		public Phase current;
		public void Init(Phase initial) 
		{
			current = initial;
		}
		public void Set(Phase next) 
		{
			current = next;
		}
		public void Tick(FishingPhysics2D context, float deltaTime)
		{
			switch (current)
			{
				case Phase.Idle:
					if (Input.GetMouseButtonDown(0))
						context.BeginCharging();
					break;
				case Phase.Charging:
					context.TickCharging(deltaTime);
					if (Input.GetMouseButtonUp(0))
						context.TickCharging();
					break;
				case Phase.Whip:
					context.TickWhip(deltaTime);
					break;
				case Phase.Flight:
					context.TickFlight(deltaTime);
					context.TryBeginReelFromFlight();
					break;
				case Phase.Landed:
					if (Input.GetMouseButtonDown(0)) 
						context.BeginReel();
					break;
				case Phase.Reeling:
					context.TickReeling(deltaTime);
					break;
			}
		}
	}
}
