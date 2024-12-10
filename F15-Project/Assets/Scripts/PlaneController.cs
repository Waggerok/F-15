using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utilities
{
    public static float MoveTo(float value, float target, float speed, float deltaTime, float min = 0, float max = 1)
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
    [SerializeField] private bool _airbrakeDeployed;//�������� �� ��������� ������
    public bool AirbrakeDeployed
    {
        get { return _airbrakeDeployed; }
        private set { _airbrakeDeployed = value; }
    }

    [Header("GameObjects")]
    [SerializeField] private List<Collider> _landingGear;//������ ����������� ��� ������� �����
    [SerializeField] private Rigidbody _rigidbody;
    [SerializeField] private PhysicMaterial _landingGearBrakesMaterial;//�������� ������ ������� ��� ������� �����, ����� ����������� ��������� ������
    [SerializeField] private PhysicMaterial _landingGearDefaultMaterial;//�������� ������ ������� ��� ������� �����, ����� �� ����������� ��������� ������

    [Header("BaseParameters")]
    [SerializeField] private float _maxThrust;//������������ ����

    [SerializeField] private float _throttleSpeed;//�������� ��������� �������� ����

    [Header("Lift")]
    [SerializeField] private float _liftPower;
    [SerializeField] private AnimationCurve _angleOfAttackLiftCurve;

    [SerializeField] private float _inducedDrag;
    [SerializeField] private AnimationCurve _inducedDragCurve;

    [SerializeField] private float _rudderPower;
    [SerializeField] private AnimationCurve _angleOfAttackRudderCurve;
    [SerializeField] private AnimationCurve _rudderInducedDragCurve;

    [SerializeField] private float _flapsLiftPower;
    [SerializeField] private float _flapsAngleOfAttackBias;
    [SerializeField] float _flapsDrag;
    [SerializeField] private float _flapsRetractSpeed;

    [Header("Drag")]
    [SerializeField] AnimationCurve _dragForward;
    [SerializeField] AnimationCurve _dragBack;
    [SerializeField] AnimationCurve _dragLeft;
    [SerializeField] AnimationCurve _dragRight;
    [SerializeField] AnimationCurve _dragTop;
    [SerializeField] AnimationCurve _dragBottom;

    [SerializeField] float _airbrakeDrag;

    [SerializeField] private bool _flapsDeployed;

    [Header("Air Density And Temperature")]
    private const float _seaLevelTemperature = 288.15f;
    private const float _temperatureLapseRate = -0.0065f;

    private Vector3 _currentVelocity;
    private Vector3 _currentLocalVelocity;
    private Vector3 _currentLocalAngularVelocity;

    private float _angleOfAttack;
    private float _angleOfAttackYaw;

    private Vector3 _localGForce;
    private Vector3 _lastVelocity;

    private float _throttle;
    public float Throttle
    {
        get { return _throttle; }
        private set { _throttle = value; }
    }
    private float _throttleInput;

    [Header("Steering")]
    [SerializeField] private Vector3 _turnSpeed;
    [SerializeField] private Vector3 _turnAcceleration;
    [SerializeField] private AnimationCurve _steeringCurve;
    [SerializeField] private Vector3 _currentControlInput;

    [SerializeField] private float _gLimit;
    [SerializeField] private float _gLimitPitch;

    private Vector3 _effectiveInput;

    
    public Vector3 EffectiveInput
    {
        get { return _effectiveInput; }
        private set { _effectiveInput = value; } 
    }

    public bool FlapsDeployed
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


    private void FixedUpdate()
    {
        var deltaTime = Time.fixedDeltaTime;

        CalculateState();
        CalculateGForces(deltaTime);
        UpdateFlaps();

        UpdateLift();
        UpdateThrust();
        UpdateThrottle(deltaTime);
        UpdateSteering(deltaTime);

        UpdateDrag();
    }

    private void CalculateState()
    {
        var invRotation = Quaternion.Inverse(_rigidbody.rotation);

        _currentVelocity = _rigidbody.velocity;
        _currentLocalVelocity = invRotation * _currentVelocity;
        _currentLocalAngularVelocity = invRotation * _rigidbody.angularVelocity;

        CalculateAngleOfAttack();
    }

    private void CalculateAngleOfAttack()
    {
        if (_currentLocalVelocity.sqrMagnitude <= 0.1f)
        {
            _angleOfAttack = 0;
            _angleOfAttackYaw = 0;

            return;
        }

        _angleOfAttack = Mathf.Atan2(-_currentLocalVelocity.y, _currentLocalVelocity.z);
        _angleOfAttackYaw = Mathf.Atan2(_currentLocalVelocity.x, _currentLocalVelocity.z);

        Debug.Log($"Velocity: {_currentLocalVelocity}, AoA: {_angleOfAttack * Mathf.Rad2Deg}");
    }

    private void CalculateGForces(float deltaTime)
    {
        var invRotation = Quaternion.Inverse(_rigidbody.rotation);
        var acceleration = (_currentVelocity - _lastVelocity) / deltaTime;

        _localGForce = invRotation * acceleration;

        _lastVelocity = _currentVelocity;
    }

    private void UpdateThrust()
    {
        var thrust = _throttle * _maxThrust;
        _rigidbody.AddRelativeForce(thrust * Vector3.forward);
    }

    public void SetThrottleInput(float input)
    {
        _throttleInput = input;
    }

    private void UpdateThrottle(float dt)
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

    private void UpdateDrag()
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

    public void ToggleFlaps()
    {
        if (_currentLocalVelocity.z < _flapsRetractSpeed)
        {
            FlapsDeployed = !FlapsDeployed;
        }
    }

    private void UpdateFlaps()
    {
        if (_currentLocalVelocity.z > _flapsRetractSpeed)
        {
            FlapsDeployed = false;
        }
    }

    private Vector3 CalculateLift(float angleOfAttack, Vector3 rightAxis, float liftPower,
        AnimationCurve animationCurve, AnimationCurve inducedDragCurve)
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

    private float CalculateAirDensity(float altitude)
    {
        var temperature = _seaLevelTemperature + _temperatureLapseRate * altitude;
        var pressure = 101325 * Mathf.Pow(1 + (_temperatureLapseRate * altitude) / _seaLevelTemperature, -9.80665f / (_temperatureLapseRate * 287.05f));
        var density = pressure / (287.05f * temperature);

        return Mathf.Max(density, 0.1f);
    }

    private void UpdateLift()
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

    private float CalculateSteering(float dt, float angularVelocity, float targetVelocity, float acceleration)
    {
        var difference = targetVelocity - angularVelocity;
        var accel = acceleration * dt;

        return Mathf.Clamp(difference, -accel, accel);
    }

    private void UpdateSteering(float dt)
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

        //��������� ������ ��������
        EffectiveInput = new Vector3(
            Mathf.Clamp(effectiveInput.x, -1, 1),
            Mathf.Clamp(effectiveInput.y, -1, 1),
            Mathf.Clamp(effectiveInput.z, -1, 1)
        );
    }

    public void SetControlInput(Vector3 input)
    {
        _currentControlInput = Vector3.ClampMagnitude(input, 1);
    }

    private Vector3 CalculateGForceLimit(Vector3 input)
    {
        return Utilities.Scale6(input,
            _gLimit, _gLimitPitch,
            _gLimit, _gLimit,
            _gLimit, _gLimit) * 9.81f;
    }

    private Vector3 CalculateGForce(Vector3 angularVelocity, Vector3 velocity)
    {
        return Vector3.Cross(angularVelocity, velocity);
    }

    private float CalculateGLimiter(Vector3 controlInput, Vector3 maxAngularVelocity)
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
}