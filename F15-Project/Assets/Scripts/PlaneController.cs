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
    public bool AirBrakeDeployed => _airbrakeDeployed;

    [Header("GameObjects")]
    [SerializeField] private List<Collider> _landingGear;
    [SerializeField] private Rigidbody _rigidbody;
    [SerializeField] private PhysicMaterial _landingGearBrakesMaterial;
    [SerializeField] private PhysicMaterial _landingGearDefaultMaterial;

    [Header("BaseParameters")]
    [SerializeField] private float _maxThrust;

    [SerializeField] private float _throttleSpeed;

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
    [SerializeField] private bool _airbrakeDeployed;

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
    private float _throttleInput;


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

    private void CalculateState()
    {
        var invRotation = Quaternion.Inverse(_rigidbody.rotation);

        _currentVelocity = _rigidbody.velocity;
        _currentLocalVelocity = invRotation * _currentVelocity;
        _currentLocalAngularVelocity = invRotation * _rigidbody.angularVelocity;

        CalculateAngleOfAttack();
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

    private void FixedUpdate()
    {
        var deltaTime = Time.fixedDeltaTime;

        CalculateState();
        CalculateGForces(deltaTime);
        UpdateFlaps();

        UpdateLift();
        UpdateThrust();
        UpdateThrottle(deltaTime);

        UpdateDrag();
    }
}