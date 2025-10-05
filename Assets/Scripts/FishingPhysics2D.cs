// =========================================================
// FishingPhysics2D.cs — Unity 6 (6000.x)  重构版（完整替换）
// 关键修复：
//  - 竿梢速度在 FixedUpdate 采样并注入（不再出现 vTip=0）
//  - 释放后让整条鱼竿保持“前倾”静止，直到收线或复位
//  - Flight 严格“松绳上限” + 早落水（绷紧若干帧 / 离屏）
//  - 随时左键可“强制开始收线”（不必等超时）
//  - Launch Feel：tipBoost / extraBoost / powerCurve 可调
// Inspector 字段名保持与你现场一致
// =========================================================

using UnityEngine;
using UnityEngine.UI;

[DisallowMultipleComponent]
[RequireComponent(typeof(LineRenderer))]
public class FishingPhysics2D : MonoBehaviour
{
	// ---------- Inspector（名称保持原样，兼容现有场景） ----------
	[Header("UI & Layers")]
	public Slider chargeSlider;
	public LayerMask waterMask;

	[Header("Rod Chain (refs)")]
	public Rigidbody2D seg1, seg2, seg3, rodTip;
	public HingeJoint2D hinge1, hinge2, hinge3;

	[Header("Bobber & Rope (refs)")]
	public Rigidbody2D bobber;            // 浮标（CircleCollider2D）
	public DistanceJoint2D rope;          // 挂在 bobber 上，connectedBody=rodTip

	[Header("Cast Tuning")]
	[Min(0.01f)] public float minCast = 1.5f;
	[Min(0.01f)] public float maxCast = 8f;
	[Tooltip("每秒蓄力进度 0..1")] public float chargeSpeed = 1.6f;
	[Tooltip("用于估计额外前向速度的时间窗")] public float flightTime = 0.35f;
	[Tooltip("最小飞行时间（早于此不落水判定）")] public float landTimeout = 0.35f;

	[Header("Drag (Unity 6 uses linearDamping)")]
	public float airDrag = 0.4f;
	public float waterDrag = 4f;

	[Header("Reel Tuning")]
	[Tooltip("distance 每秒缩短量")] public float reelRate = 6f;
	[Tooltip("沿绳方向额外拉力")] public float reelForce = 12f;

	[Header("Whip（甩竿）")]
	public float whipMotorSpeed = 1400f;
	public float whipPulse = 0.10f;
	public bool releaseAtForwardApex = true;

	[Header("Flight Gravity")]
	[Tooltip("飞行期的重力系数（顶视角 0.10~0.30）")] public float flightGravity = 0.15f;

	[Header("Launch Feel（手感增强）")]
	[Tooltip("竿梢速度增益")] public float tipBoost = 1.6f;
	[Tooltip("解析额外速度增益")] public float extraBoost = 1.1f;
	[Tooltip("蓄力曲线 >1 变硬，<1 变软")] public float powerCurve = 1.15f;

	[Header("Flight Lock（视觉友好落水）")]
	[Tooltip("飞行允许的额外松弛比例")][Range(0f, 0.5f)] public float slackRatio = 0.08f;
	[Tooltip("连续多少帧“已绷紧”就直接落水")][Range(1, 8)] public int tautFramesToLand = 2;
	[Tooltip("离屏外溢容差（0~0.2）")][Range(0f, 0.2f)] public float offscreenMargin = 0.04f;

	[Header("Line Renderer")]
	[Min(2)] public int lineSegments = 16;
	[Min(16)] public int pixelsPerUnit = 128;
	public int sortingOrder = 10;
	public Color lineColor = new Color(1f, 0.1f, 0.6f, 1f);
	public bool drawDebugLine = true;

	[Header("Home Pose（复位外观）")]
	public bool snapRodOnReelFinish = true;
	public bool snapRodAlsoFromEmergencyReset = true;

	// ------------------------- 内部状态 -------------------------
	enum Phase { Idle, Charging, Whip, Flight, Landed, Reeling }
	Phase phase = Phase.Idle;

	LineRenderer lr;
	float charge01, castTimer, plannedLen;
	float uiFadeT; bool uiFading;
	float whipT, tipFwdPrev;             // 甩竿计时 & 前向度量
	int tautFrames;                      // 连续“已绷紧”计数

	// 竿梢“真实”速度（由 Seg3 刚体在 FixedUpdate 计算）
	Vector2 tipVelFixed;

	// Home Pose 快照
	struct TPose
	{
		public Vector3 pos; public Quaternion rot; public Vector3 scale;
		public TPose(Transform t) { pos = t.localPosition; rot = t.localRotation; scale = t.localScale; }
		public void Apply(Transform t) { t.localPosition = pos; t.localRotation = rot; t.localScale = scale; }
	}
	TPose seg1Home, seg2Home, seg3Home, tipHome, bobberHome;
	bool homeCaptured;

	// ------------------------- 生命周期 -------------------------
	void Awake()
	{
		lr = GetComponent<LineRenderer>();
		lr.useWorldSpace = true;
		if (!lr.material) lr.material = new Material(Shader.Find("Sprites/Default"));
		lr.startColor = lr.endColor = lineColor;
		lr.sortingOrder = sortingOrder;
		ConfigureLine();

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
		// 右键紧急复位
		if (Input.GetMouseButtonDown(1)) { ResetAll(snapRodAlsoFromEmergencyReset); return; }

		switch (phase)
		{
			case Phase.Idle:
				if (Input.GetMouseButtonDown(0)) BeginCharging();
				break;

			case Phase.Charging:
				TickCharging(Time.deltaTime);
				if (Input.GetMouseButtonUp(0)) BeginWhip();
				break;

			case Phase.Whip:
				TickWhip(Time.deltaTime);
				break;

			case Phase.Flight:
				TickFlight(Time.deltaTime);
				TryBeginReelFromFlight();
				break;

			case Phase.Landed:
				if (Input.GetMouseButtonDown(0)) BeginReel();
				break;

			case Phase.Reeling:
				TickReeling(Time.deltaTime);
				break;
		}

		// Pre-Release：刚性整体
		EnsureAttachedDuringPreRelease();

		TickUiFade(Time.deltaTime);
		UpdateLineRenderer();
		Watchdog();
	}

	// --------------------------- 状态实现 ---------------------------
	void BeginCharging()
	{
		phase = Phase.Charging;
		charge01 = 0f; castTimer = 0f;

		if (chargeSlider) { chargeSlider.value = 0; chargeSlider.gameObject.SetActive(true); }

		AttachToTipKeepWorld();
		bobber.transform.localPosition = Vector3.zero;
		if (rope) rope.enabled = false;

		Debug.Log("[Fishing] Charging start", this);
	}

	void TickCharging(float dt)
	{
		charge01 = Mathf.Min(1f, charge01 + Mathf.Max(0f, chargeSpeed) * dt);
		if (chargeSlider) chargeSlider.value = charge01;
	}

	void BeginWhip()
	{
		phase = Phase.Whip;
		whipT = 0f; tipFwdPrev = -999f;
		plannedLen = minCast + (maxCast - minCast) * Mathf.Pow(Mathf.Clamp01(charge01), Mathf.Max(0.05f, powerCurve));
		if (chargeSlider) StartUiFadeOut();
		PulseHingeTowardMouse();
		Debug.Log($"[Fishing] WHIP charge={charge01:F2} lenPlan={plannedLen:F2}", this);
	}

	void TickWhip(float dt)
	{
		whipT += dt;
		bobber.transform.localPosition = Vector3.zero;

		Vector2 origin = rodTip ? (Vector2)rodTip.position : (Vector2)transform.position;
		Vector2 mouse = Camera.main ? (Vector2)Camera.main.ScreenToWorldPoint(Input.mousePosition) : origin + Vector2.right;
		Vector2 castDir = (mouse - origin).sqrMagnitude > 1e-6f ? (mouse - origin).normalized : Vector2.right;

		// 用 FixedUpdate 缓存的真实竿梢速度
		Vector2 vTip = tipVelFixed;
		float tipFwd = (vTip.sqrMagnitude < 1e-6f) ? -999f : Vector2.Dot(vTip.normalized, castDir);
		bool apex = releaseAtForwardApex && tipFwdPrev > -998f && tipFwdPrev > tipFwd; // 前向速度开始回落
		tipFwdPrev = tipFwd;

		if (apex || whipT >= whipPulse) ReleaseNow(castDir, vTip);
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

		phase = Phase.Flight;
		castTimer = 0f; tautFrames = 0;

		Debug.Log($"[Fishing] RELEASE vTip={vTip.magnitude:F2} v0={v0:F2}", this);
	}

	void TickFlight(float dt)
	{
		castTimer += dt;
		if (!bobber || !rope || !rodTip) return;

		// 视口外溢：强制落水（避免“飞出屏外还不落”）
		if (IsOffscreen(bobber.position, offscreenMargin)) { ForceLand(); return; }

		float tipTo = Vector2.Distance(rodTip.position, bobber.position);
		bool tight = tipTo >= rope.distance - 0.01f;
		tautFrames = tight ? (tautFrames + 1) : 0;

		// 拉紧若干帧就落水（视觉友好）
		if (tautFrames >= Mathf.Max(1, tautFramesToLand) && castTimer >= landTimeout * 0.4f) { ForceLand(); return; }

		// 到时后落水：命中水面，或已绷紧且速度回向竿梢
		if (castTimer >= landTimeout)
		{
			bool inWater = Physics2D.OverlapPoint(bobber.position, waterMask) != null;
			Vector2 toTip = (Vector2)rodTip.position - bobber.position;
			bool inward = Vector2.Dot(toTip, bobber.linearVelocity) > 0f
						  || bobber.linearVelocity.sqrMagnitude < 0.0004f;
			if (inWater || (tight && inward)) ForceLand();
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
		if (rope) { rope.enabled = true; rope.maxDistanceOnly = true; rope.distance = cur; }
		bobber.linearVelocity = Vector2.zero;
		bobber.angularVelocity = 0f;
		bobber.linearDamping = waterDrag;
		bobber.gravityScale = 0f;
		phase = Phase.Landed;
		Debug.Log($"[Fishing] LANDED lockLen={cur:F2}", this);
	}

	void TryBeginReelFromFlight()
	{
		if (!Input.GetMouseButtonDown(0) || phase != Phase.Flight || rope == null || rodTip == null) return;
		if (castTimer < landTimeout) return;
		float tipTo = Vector2.Distance(rodTip.position, bobber.position);
		if (tipTo < rope.distance - 0.02f) return; // 未绷紧
		ForceLand();
		BeginReel();
		Debug.Log("[Fishing] Begin reel from Flight (tight & timeout)", this);
	}

	void BeginReel()
	{
		phase = Phase.Reeling;
		if (rope) rope.maxDistanceOnly = false; // 绝对长度模式：distance 变短会“拖回来”
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

				phase = Phase.Idle;
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
		AttachToTipKeepWorld();
		bobber.transform.localPosition = Vector3.zero;
	}

	void ResetAll(bool snapRod)
	{
		phase = Phase.Idle;
		charge01 = 0f; castTimer = 0f; plannedLen = 0f;
		uiFading = false; uiFadeT = 0f; whipT = 0f; tipFwdPrev = -999f; tautFrames = 0;
		tipVelFixed = Vector2.zero;

		if (rope) { rope.enabled = false; rope.distance = 0f; rope.maxDistanceOnly = true; }

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

		if (chargeSlider) { chargeSlider.value = 0f; chargeSlider.gameObject.SetActive(false); }

		if (snapRod) RestoreHomePoseInstant();

		lr.enabled = false;
		UpdateLineRenderer();

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
			ResetAll(true);
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
}
