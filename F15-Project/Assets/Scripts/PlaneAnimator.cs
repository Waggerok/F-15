using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneAnimator : MonoBehaviour
{
    private void LateUpdate()
    {
        var dt = Time.deltaTime;

        UpdateAfterburners();
        UpdateControlSurfaces(dt);
        UpdateAirbrakes(dt);
        UpdateFlaps(dt);
    }

    private void Start()
    {
        AddNeutralPose(_leftAileron);
        AddNeutralPose(_rightAileron);

        foreach (var t in _elevators)
        {
            AddNeutralPose(t);
        }

        foreach (var t in _rudders)
        {
            AddNeutralPose(t);
        }

        AddNeutralPose(_airbrake);

        foreach (var t in _flaps)
        {
            AddNeutralPose(t);
        }
    }
    private void UpdateAfterburners()
    {
        var throttle = _planeController.Throttle;

        var afterburnerT = Mathf.Clamp01(Mathf.InverseLerp(_afterBurnerTreshold, 1, throttle));

        var size = Mathf.Lerp(_afterburnerMinSize, _afterburnerMaxSize, afterburnerT);

        if (throttle >= _afterBurnerTreshold)
        {
            for (var i = 0; i < _afterburnerGraphics.Count; i++)
            {
                _afterburnerGraphics[i].SetActive(true);
                _afterburnersTransforms[i].localScale = new Vector3(size, size, size);
            }
        }
        else
        {
            foreach (var afterBurner in _afterburnerGraphics)
            {
                afterBurner.SetActive(false);
            }
        }
    }

    private void UpdateControlSurfaces(float dt)
    {
        var input = _planeController.EffectiveInput;

        _deflection.x = Utilities.MoveTo(_deflection.x, input.x, _deflectionSpeed, dt, -1, 1);
        _deflection.y = Utilities.MoveTo(_deflection.y, input.y, _deflectionSpeed, dt, -1, 1);
        _deflection.z = Utilities.MoveTo(_deflection.z, input.z, _deflectionSpeed, dt, -1, 1);

        Debug.LogWarning($"Deflection: {_deflection.x}, {_deflection.y}, {_deflection.z}");

        _leftAileron.localRotation = CalculatePose(_leftAileron, Quaternion.Euler(-_deflection.z * _maxAileronDeflection, 0, 0));
        _rightAileron.localRotation = CalculatePose(_rightAileron, Quaternion.Euler(_deflection.z * _maxAileronDeflection, 0, 0));

        foreach (var t in _elevators)
        {
            t.localRotation = CalculatePose(t, Quaternion.Euler(_deflection.x * _maxElevatorDeflection, 0, 0));
        }

        foreach (var t in _rudders)
        {
            t.localRotation = CalculatePose(t, Quaternion.Euler(0, -_deflection.y * _maxRudderDeflection, 0));
        }
    }

    private Quaternion CalculatePose(Transform transform, Quaternion offset)
    {
        return _neutralPoses[transform] * offset;
    }

    private void AddNeutralPose(Transform transform)
    {
        _neutralPoses.Add(transform, transform.localRotation);
    }

    private void UpdateAirbrakes(float dt)
    {
        var target = _planeController.AirbrakeDeployed ? 1 : 0;
        _airbrakePosition = Utilities.MoveTo(_airbrakePosition, target, _deflectionSpeed, dt);

        _airbrake.localRotation = CalculatePose(_airbrake, Quaternion.Euler(-_airbrakePosition * _airBrakeDeflection, 0, 0));
    }

    private void UpdateFlaps(float dt)
    {
        var target = _planeController.FlapsDeployed ? 1 : 0;
        _flapsPosition = Utilities.MoveTo(_flapsPosition, target, _deflectionSpeed, dt);

        foreach (var t in _flaps)
        {
            t.localRotation = CalculatePose(t, Quaternion.Euler(_flapsPosition * _flapsDeflection, 0, 0));
        }
    }


    [SerializeField] private PlaneController _planeController;

    [Header("AfterBurner")]
    [SerializeField] private List<GameObject> _afterburnerGraphics;
    [SerializeField] private List<Transform> _afterburnersTransforms;

    [SerializeField] private float _afterBurnerTreshold;
    [SerializeField] private float _afterburnerMinSize;
    [SerializeField] private float _afterburnerMaxSize;

    [Header("Deflection")]
    [SerializeField] private float _deflectionSpeed;

    [SerializeField] private float _maxAileronDeflection;
    [SerializeField] private float _maxElevatorDeflection;
    [SerializeField] private float _maxRudderDeflection;

    [Header("Aileron")]
    [SerializeField] private Transform _leftAileron;
    [SerializeField] private Transform _rightAileron;

    [Header("Elevators")]
    [SerializeField] private List<Transform> _elevators;

    [Header("Rudders")]
    [SerializeField] private List<Transform> _rudders;

    private Vector3 _deflection;

    private readonly Dictionary<Transform, Quaternion> _neutralPoses = new();

    [Header("AirBrake")]
    [SerializeField] private Transform _airbrake;

    [SerializeField] private float _airBrakeDeflection;
    private float _airbrakePosition;

    [Header("Flaps")]
    [SerializeField] private List<Transform> _flaps;

    [SerializeField] private float _flapsDeflection;
    private float _flapsPosition;
}


