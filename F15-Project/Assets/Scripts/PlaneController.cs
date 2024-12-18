using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utilities //нужен для плавным изменением значений, он позволит задать скорость изменения
{
    public static float MoveTo(float value, float target, float speed, float deltaTime, float min = 0, float max = 1) //
    {
        var diff = target - value;

        var delta = Mathf.Clamp(diff, -speed * deltaTime, speed * deltaTime);

        return Mathf.Clamp(value + delta, min, max);
    }

    public static Vector3 Scale6(
    Vector3 value,
    float positiveX, float negativeX,
    float positiveY, float negativeY,
    float positiveZ, float negativeZ
)   {
        var result = value;

        switch (result.x)
        {
            case > 0:
                result.x *= positiveX;
                break;
            case < 0:
                result.x *= negativeX;
                break;
        }

        switch (result.y)
        {
            case > 0:
                result.y *= positiveY;
                break;
            case < 0:
                result.y *= negativeY;
                break;
        }

        switch (result.z)
        {
            case > 0:
                result.z *= positiveZ;
                break;
            case < 0:
                result.z *= negativeZ;
                break;
        }

        return result;
    }

}

public class PlaneController : MonoBehaviour
{
    [SerializeField] private bool _airbrakeDeployed; //помогает замедлить самолёт, создавая доп. сопротивление
    public bool AirbrakeDeployed
    {
        get { return _airbrakeDeployed; }
        private set { _airbrakeDeployed = value; }
    }

    [Header("GameObjects")]
    [SerializeField] private List<Collider> _landingGear; //список коллайдеров для каждого колеса
    [SerializeField] private Rigidbody _rigidbody;
    [SerializeField] private PhysicMaterial _landingGearBrakesMaterial; //материал физики для тормозного состояния шасси
    [SerializeField] private PhysicMaterial _landingGearDefaultMaterial; //материал физики для стандартного состояния шасси

    [Header("BaseParameters")]
    [SerializeField] private float _maxThrust; // Хранит максимальную силу тяги

    [SerializeField] private float _throttleSpeed; //хранит скорость изменения значения дросселя

    [Header("Lift")]
    [SerializeField] private float _liftPower; //основная сила подъёма
    [SerializeField] private AnimationCurve _angleOfAttackLiftCurve; //анимационная кривая, описывающая зависимость подъемной силы от угла атаки

    [SerializeField] private float _inducedDrag; // задает коэффицент идуцированного сопротивления
    [SerializeField] private AnimationCurve _inducedDragCurve; //кривая, описывающая зависимость индуцированного сопротивления от подъемной силы

    [SerializeField] private float _rudderPower; //задает мощность управляющего руля
    [SerializeField] private AnimationCurve _angleOfAttackRudderCurve; //кривая, описывающая , как подъемная сила
    [SerializeField] private AnimationCurve _rudderInducedDragCurve;//кривая, описывающая индуцированное сопротивление

    [SerializeField] private float _flapsLiftPower; //задает доп. подъемную силу, при развернутых закрылках
    [SerializeField] private float _flapsAngleOfAttackBias; //хранит изменение угла атаки при развернутых закрылках
    [SerializeField] float _flapsDrag; //хранит дополнительное сопротивление, создаваемое закрылками
    [SerializeField] private float _flapsRetractSpeed; //хранит скорость, при достижении которой закрылки автоматически убираются

    [Header("Drag")] //Изменение сопротивление при движении
    [SerializeField] AnimationCurve _dragForward;
    [SerializeField] AnimationCurve _dragBack;
    [SerializeField] AnimationCurve _dragLeft;
    [SerializeField] AnimationCurve _dragRight;
    [SerializeField] AnimationCurve _dragTop;
    [SerializeField] AnimationCurve _dragBottom;

    [SerializeField] float _airbrakeDrag; //задает величину дополнительного сопротивления

    [SerializeField] private bool _flapsDeployed; //true|false переменная, развернуты ли закрылки на истребителе

    [Header("Air Density And Temperature")]
    private const float _seaLevelTemperature = 288.15f; //температура на уровне моря в Кельвинах
    private const float _temperatureLapseRate = -0.0065f; //коэффицент изменения температуры в зависимости от высоты

    private Vector3 _currentVelocity; //  хранит текущую мировую скорость самолета
    private Vector3 _currentLocalVelocity; // Поле которое хранит текущую локальную скорость самолета
    private Vector3 _currentLocalAngularVelocity; // Это поле хранит текущую локальную угловую скорость самолета

    
    private float _angleOfAttack; // хранит значение угла атаки вдоль продольной оси
    private float _angleOfAttackYaw; // хранит значение угла атаки вдоль горизонтальной оси

    private Vector3 _localGForce; //хранит текущие перегузки самолета
    private Vector3 _lastVelocity; //хранит значение скорости самолета на предыдущем кадре

    private float _throttle; // хранит сколько тяги в данный момент прикладывается к самолёту
    public float Throttle
    {
        get { return _throttle; }
        private set { _throttle = value; }
    }
    private float _throttleInput; // Это поле задает текущий пользовательский ввод для дросселя.

    [Header("Steering")]
    [SerializeField] private Vector3 _turnSpeed; //определяет максимальные скорости поворота 
    [SerializeField] private Vector3 _turnAcceleration; // определяет максимальные ускорения
    [SerializeField] private AnimationCurve _steeringCurve; //анимационная кривая, задает силу поворота в зависимости от текущей скорости самолета
    [SerializeField] private Vector3 _currentControlInput; //хранит текущее управление от игрока

    [SerializeField] private float _gLimit; //задает максимальную допустимую перегрузку по осям X и Z
    [SerializeField] private float _gLimitPitch; //задает максимальную допустимую перегрузку по оси Y

    private Vector3 _effectiveInput;

    [SerializeField] private List<GameObject> _graphicsObjects;

    private bool _isDestroyed = false;

    [SerializeField] private float _baseMass = 14300;

    [Header("Fuel")]
    [SerializeField] private float _fuelCapacity = 6100f;
    [SerializeField] private float _fuelConsumptionRate = 0.5f;
    [SerializeField] private float _fuelDensity = 0.8f;

    [SerializeField] private float _fuelAmount;

    private bool _engineOff = false;

    [Header("Turbulence")]
    [SerializeField] private float _turbulenceStrength = 0.05f;
    [SerializeField] private float _turbulenceFrequency = 1f;

    [Header("Stall")]
    [SerializeField] private float _stallAngle = 15f;
    [SerializeField] private float _stallRecoveryForce = 10f;
    [SerializeField] private float _stallTorqueMultiplier = 1f;

    [SerializeField] private float _initialSpeed = 50f; // Начальная скорость, можете задать значение по умолчанию

    private bool _isStalled = false;

    [Header("Glide")]
    [SerializeField] private float _glideRotationSpeed = 30f;
    [SerializeField] private float _glideDamping = 0.5f;

    [Header("Angular Drag")]
    [SerializeField] private Vector3 _angularDrag = new Vector3(1f, 1f, 1f);

    public Vector3 EffectiveInput
    {
        get { return _effectiveInput; }
        private set { _effectiveInput = value; } 
    }

    public bool FlapsDeployed //свойство, которое позволяет управлять состоянием закрылок и шасси
    {
        get => _flapsDeployed;

        private set
        {
            _flapsDeployed = value;

            foreach (var lg in _landingGear)
            {
                lg.enabled = value;
            }
        }
    }


    private void FixedUpdate() // Для просчета физических сил
    {
        var deltaTime = Time.fixedDeltaTime; // сохранение фиксированного интервала времени в переменную

        CalculateState();
        CalculateGForces(deltaTime);
        UpdateFlaps();

        UpdateThrottle(deltaTime);

        UpdateFuel(deltaTime);

        if (!_isDestroyed && !_engineOff)
        {
            ApplyTurbulence();

            UpdateThrust();
            UpdateLift();
            UpdateSteering(deltaTime);

            HandleStall();
        }
        if (_engineOff)
        {
            Glide(); //переход в режим планирования
        }

        UpdateDrag();
        UpdateAngularDrag();
    }

    private void CalculateState() // Рассчитывание состояние самолёта
    {
        var invRotation = Quaternion.Inverse(_rigidbody.rotation);

        _currentVelocity = _rigidbody.velocity;
        _currentLocalVelocity = invRotation * _currentVelocity;
        _currentLocalAngularVelocity = invRotation * _rigidbody.angularVelocity;

        CalculateAngleOfAttack();
    }

    private void CalculateAngleOfAttack() // Рассчитывание угла атаки истребителя
    {
        if (_currentLocalVelocity.sqrMagnitude <= 0.1f)
        {
            _angleOfAttack = 0;
            _angleOfAttackYaw = 0;

            _isStalled = Mathf.Abs(_angleOfAttack) > _stallAngle; //добавляем строчки для расчета состояния сваливания

            Debug.Log($"Velocity: {_currentLocalVelocity}, AoA: {_angleOfAttack}°, IsStalled: {_isStalled}"); 

            return;
        }

        _angleOfAttack = Mathf.Atan2(-_currentLocalVelocity.y, _currentLocalVelocity.z);
        _angleOfAttackYaw = Mathf.Atan2(_currentLocalVelocity.x, _currentLocalVelocity.z);

        Debug.Log($"Velocity: {_currentLocalVelocity}, AoA: {_angleOfAttack * Mathf.Rad2Deg}");
    }

    private void CalculateGForces(float deltaTime) // Рассчитывание перегрузки истребителя
    {
        var invRotation = Quaternion.Inverse(_rigidbody.rotation);
        var acceleration = (_currentVelocity - _lastVelocity) / deltaTime;

        _localGForce = invRotation * acceleration;

        _lastVelocity = _currentVelocity;
    }

    private void UpdateThrust() //Метод который применяет тягу к самолёту
    {
        var thrust = _throttle * _maxThrust;
        _rigidbody.AddRelativeForce(thrust * Vector3.forward);
    }

    public void SetThrottleInput(float input) // этот метод меняет _throttleInput в зависимости от ввода пользователя
    {
        _throttleInput = input;
    }

    private void UpdateThrottle(float dt) // Обновляет значение дросселя
    {
        float target = 0;

        if (_throttleInput > 0)
        {
            target = 1;
        }

        _throttle = Utilities.MoveTo(_throttle, target, _throttleSpeed * Mathf.Abs(_throttleInput), dt);

        _airbrakeDeployed = _throttle == 0 && _throttleInput == -1;

        if (_airbrakeDeployed)
        {
            foreach (var lg in _landingGear)
            {
                lg.sharedMaterial = _landingGearBrakesMaterial;
            }
        }
        else
        {
            foreach (var lg in _landingGear)
            {
                lg.sharedMaterial = _landingGearDefaultMaterial;
            }
        }
    }

    private void UpdateDrag() //рассчитывает силу сопротивления воздуха и применение её к самолёту
    {
        var globalVelocity = _rigidbody.velocity;
        var globalVelocitySqr = globalVelocity.sqrMagnitude;

        var airbrakeDrag = _airbrakeDeployed ? _airbrakeDrag : 0f;
        var flapsDrag = _flapsDeployed ? _flapsDrag : 0f;

        var dragCoefficientToDirections = Utilities.Scale6(
            globalVelocity.normalized,
            _dragRight.Evaluate(Mathf.Abs(globalVelocity.x)),
            _dragLeft.Evaluate(Mathf.Abs(globalVelocity.x)),
            _dragTop.Evaluate(Mathf.Abs(globalVelocity.y)),
            _dragBottom.Evaluate(Mathf.Abs(globalVelocity.y)),
            _dragForward.Evaluate(Mathf.Abs(globalVelocity.z) + airbrakeDrag + flapsDrag),
            _dragBack.Evaluate(Mathf.Abs(globalVelocity.z))
        );

        var drag = dragCoefficientToDirections.magnitude * globalVelocitySqr * -globalVelocity.normalized;

        _rigidbody.AddForce(drag, ForceMode.Force);
    }

    public void ToggleFlaps() //переключает состояние закрылок в зависимости от скорости самолета
    {
        if (_currentLocalVelocity.z < _flapsRetractSpeed)
        {
            FlapsDeployed = !FlapsDeployed;
        }
    }

    private void UpdateFlaps() //автоматически убирает закрылки при достижении скорости в 125 единиц
    {
        if (_currentLocalVelocity.z > _flapsRetractSpeed)
        {
            FlapsDeployed = false;
        }
    }

    private Vector3 CalculateLift(float angleOfAttack, Vector3 rightAxis, float liftPower, //рассчитывает силу подъёма и индуцированное сопротивление на основе текущего угла атаки, 
        AnimationCurve animationCurve, AnimationCurve inducedDragCurve) // скорости и плотности воздухаф
    {
        var liftVelocityOnWings = Vector3.ProjectOnPlane(_currentLocalVelocity, rightAxis);
        var liftVelocityOnWingsSqr = liftVelocityOnWings.sqrMagnitude;
        var liftCoefficient = animationCurve.Evaluate(angleOfAttack * Mathf.Rad2Deg);
        var liftForce = liftVelocityOnWingsSqr * liftCoefficient * liftPower;
        var liftDirection = Vector3.Cross(liftVelocityOnWings.normalized, rightAxis);
        var lift = liftDirection * liftForce;

        var dragForce = liftCoefficient * liftCoefficient * _inducedDrag;
        var dragDirection = -liftVelocityOnWings.normalized;
        var inducedDrag = dragDirection * liftVelocityOnWingsSqr * dragForce
                          * inducedDragCurve.Evaluate(Mathf.Max(0, _currentLocalVelocity.z));

        return lift + inducedDrag;
    }

    private float CalculateAirDensity(float altitude) //расчет плотности воздуха в зависимости от высоты
    {
        var temperature = _seaLevelTemperature + _temperatureLapseRate * altitude;
        var pressure = 101325 * Mathf.Pow(1 + (_temperatureLapseRate * altitude) / _seaLevelTemperature, -9.80665f / (_temperatureLapseRate * 287.05f));
        var density = pressure / (287.05f * temperature);

        return Mathf.Max(density, 0.1f);
    }

    private void UpdateLift() //просчитывает силу подъема
    {
        if (_currentLocalVelocity.sqrMagnitude < 1f)
        {
            return;
        }

        var airDensity = CalculateAirDensity(_rigidbody.position.y);

        var flapsLiftPower = FlapsDeployed ? _flapsLiftPower : 0;
        var flapsAngleOfAttackBias = FlapsDeployed ? _flapsAngleOfAttackBias : 0;

        var effectiveAoA = _angleOfAttack + (flapsAngleOfAttackBias * Mathf.Deg2Rad);

        var liftMultiplier = 1f;

        var liftForce = CalculateLift(effectiveAoA,
            Vector3.right, (_liftPower + flapsLiftPower) * airDensity * liftMultiplier,
            _angleOfAttackLiftCurve, _inducedDragCurve);

        var yawForce = CalculateLift(_angleOfAttackYaw, Vector3.up, _rudderPower * airDensity,
            _angleOfAttackRudderCurve, _rudderInducedDragCurve);

        _rigidbody.AddRelativeForce(liftForce);
        _rigidbody.AddRelativeForce(yawForce);
    }

    private float CalculateSteering(float dt, float angularVelocity, float targetVelocity, float acceleration)//отвечает за корректировку угловой скорости самолета с учетом текущего состояния
    {
        var difference = targetVelocity - angularVelocity;
        var accel = acceleration * dt;

        return Mathf.Clamp(difference, -accel, accel);
    }

    private void UpdateSteering(float dt) //рассчитывает и применяет силу поворота к самолету
    {
        var speed = Mathf.Max(0, _currentLocalVelocity.z);
        var steeringPower = _steeringCurve.Evaluate(speed);
        var airDensity = CalculateAirDensity(_rigidbody.position.y);

        var gForceScaling = CalculateGLimiter(_currentControlInput, _turnSpeed * Mathf.Deg2Rad * steeringPower * airDensity);

        var targetAV = Vector3.Scale(_currentControlInput, _turnSpeed * steeringPower * gForceScaling);
        var av = _currentLocalAngularVelocity * Mathf.Rad2Deg;

        var correction = new Vector3(
            CalculateSteering(dt, av.x, targetAV.x, _turnAcceleration.x * steeringPower),
            CalculateSteering(dt, av.y, targetAV.y, _turnAcceleration.y * steeringPower),
            CalculateSteering(dt, av.z, targetAV.z, _turnAcceleration.z * steeringPower));

        _rigidbody.AddRelativeTorque(correction * Mathf.Deg2Rad, ForceMode.VelocityChange);

        var correctionInput = new Vector3(
            Mathf.Clamp((targetAV.x - av.x) / _turnAcceleration.x, -1, 1),
            Mathf.Clamp((targetAV.y - av.y) / _turnAcceleration.y, -1, 1),
            Mathf.Clamp((targetAV.z - av.z) / _turnAcceleration.z, -1, 1)
        );

        var effectiveInput = (correctionInput + _currentControlInput) * gForceScaling;

        EffectiveInput = new Vector3(
            Mathf.Clamp(effectiveInput.x, -1, 1),
            Mathf.Clamp(effectiveInput.y, -1, 1),
            Mathf.Clamp(effectiveInput.z, -1, 1)
        );
    }

    public void SetControlInput(Vector3 input) //передает управление в истребитель
    {
        _currentControlInput = Vector3.ClampMagnitude(input, 1);
    }

    private Vector3 CalculateGForceLimit(Vector3 input) //рассчитывает перегрузки на основе пользовательского ввода и ограничений
    {
        return Utilities.Scale6(input,
            _gLimit, _gLimitPitch,
            _gLimit, _gLimit,
            _gLimit, _gLimit) * 9.81f;
    }

    private Vector3 CalculateGForce(Vector3 angularVelocity, Vector3 velocity) //просчитывает calculateGForce при действии угловой скорости
    {
        return Vector3.Cross(angularVelocity, velocity);
    }

    private float CalculateGLimiter(Vector3 controlInput, Vector3 maxAngularVelocity) //вычисляет коэффицент масштабирования для управления с учетом G-Limit
    {
        var maxInput = controlInput.normalized;

        var limit = CalculateGForceLimit(maxInput);

        var maxGForce = CalculateGForce(Vector3.Scale(maxInput, maxAngularVelocity), _currentLocalVelocity);

        if (maxGForce.magnitude > limit.magnitude)
        {
            return limit.magnitude / maxGForce.magnitude;
        }

        return 1;
    }

    private void Die() //отключает физику самолета и визуальные компоненты
    {
        _throttle = 0;
        _isDestroyed = true;

        _rigidbody.isKinematic = true;

        foreach (var go in _graphicsObjects)
        {
            go.SetActive(false);
        }

        Debug.Log("Самолет разрушен.");
    }

    private void OnCollisionEnter(Collision collision) // при столкновении уничтожает истребитель
    {
        for (var i = 0; i < collision.contactCount; i++)
        {
            var contact = collision.contacts[i];

            if (_landingGear.Contains(contact.thisCollider))
            {
                return;
            }

            _rigidbody.isKinematic = true;
            _rigidbody.position = contact.point;
            _rigidbody.rotation = Quaternion.Euler(0, _rigidbody.rotation.eulerAngles.y, 0);

            foreach (var go in _graphicsObjects)
            {
                go.SetActive(false);
            }

            Die();

            return;
        }
    }

    private void Start() //первоночальная инициализации полей массы и топлива
    {
        _rigidbody.velocity = _rigidbody.rotation * new Vector3(0, 0, _initialSpeed);

        _fuelAmount = _fuelCapacity;
        UpdateMass();

        _lastVelocity = _rigidbody.velocity;
    }


    private void UpdateMass() //обновляет массу в зависимости от количества топлива
    {
        var fuelMass = _fuelAmount * _fuelDensity;
        _rigidbody.mass = _baseMass + fuelMass;
    }

    private void UpdateFuel(float deltaTime) //отвечает за расход товлива в зависимости от положения дросселя
    {
        if (_throttle > 0 && !_engineOff)
        {
            var fuelConsumed = _throttle * _fuelConsumptionRate * deltaTime;
            _fuelAmount = Mathf.Max(0.1f, _fuelAmount - fuelConsumed);

            UpdateMass();
        }

        if (_fuelAmount <= 0 && !_engineOff)
        {
            _engineOff = true;
            _fuelAmount = 0;

            Debug.Log("Топливо закончилось, двигатель выключен.");
        }
    }

    private void ApplyTurbulence() //добавляет случайные вращательные моменты к самолету, имитируя турбулентные возмущения
    {
        var turbulence = new Vector3(
            Random.Range(-_turbulenceStrength, _turbulenceStrength),
            Random.Range(-_turbulenceStrength, _turbulenceStrength),
            Random.Range(-_turbulenceStrength, _turbulenceStrength)
        );

        _rigidbody.AddRelativeTorque(turbulence, ForceMode.Acceleration);
    }

    private void HandleStall() //отвечает за попытки стабилизации при сваливании
    {
        if (!_isStalled)
        {
            return;
        }

        var stabilizationTorque = new Vector3(-_currentLocalAngularVelocity.x, -_currentLocalAngularVelocity.y, -_currentLocalAngularVelocity.z);
        _rigidbody.AddRelativeTorque(stabilizationTorque * _stallRecoveryForce, ForceMode.Acceleration);
    }

    private void Glide() //при выключенном двигателе стабилизирует ориентаию и медленно снижается
    {
        if (!_engineOff)
        {
            return;
        }

        if (_rigidbody.velocity.sqrMagnitude < 0.1f)
        {
            return;
        }

        var forward = _rigidbody.velocity.normalized;
        var up = Vector3.up;

        var targetRotation = Quaternion.LookRotation(forward, up);

        _rigidbody.rotation = Quaternion.RotateTowards(_rigidbody.rotation, targetRotation, _glideRotationSpeed * Time.deltaTime);
        _rigidbody.angularVelocity = Vector3.Lerp(_rigidbody.angularVelocity, Vector3.zero, Time.deltaTime * _glideDamping);
    }

    private void UpdateAngularDrag() //просчет углового сопротивления
    {
        var av = _currentLocalAngularVelocity;

        if (av.sqrMagnitude == 0)
        {
            return;
        }

        var drag = Vector3.Scale(av.sqrMagnitude * -av.normalized, _angularDrag);

        _rigidbody.AddRelativeTorque(drag, ForceMode.Acceleration);
    }

}