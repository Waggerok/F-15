using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerController : MonoBehaviour, PlayerInputAsset.IPlayerInputActions
{
    [SerializeField] private PlaneController _planeController;

    private PlayerInputAsset _playerInput;
    private Vector3 _controlInput;

    void Awake()
    {
        _playerInput = new PlayerInputAsset();

        _playerInput.PlayerInput.SetCallbacks(this);
    }

    // Передача значения в класс PlaneController
    private void Update()
    {
        if (_planeController == null)
        {
            return;
        }

        _planeController.SetControlInput(_controlInput);
    }

    void OnEnable()
    {
        _playerInput.PlayerInput.Enable();
    }

    void OnDisable()
    {
        _playerInput.PlayerInput.Disable();
    }

    public void OnThrottle(InputAction.CallbackContext context)
    {
        if (_planeController == null)
        {
            return;
        }

        _planeController.SetThrottleInput(context.ReadValue<float>());
    }

    public void OnFlaps(InputAction.CallbackContext context) //нужен для получения значений после нажатия кнопки F
    {
        if (_planeController == null)
        {
            return;
        }

        if (context.phase == InputActionPhase.Performed)
        {
            _planeController.ToggleFlaps();
        }
    }

    // Новый код обработки поворота 
    public void OnRollPitch(InputAction.CallbackContext context)
    {
        if (_planeController == null)
        {
            return;
        }

        var input = context.ReadValue<Vector2>();

        _controlInput = new Vector3(input.y, _controlInput.y, -input.x);
    }

    // Новый код обработки поворота 
    public void OnYaw(InputAction.CallbackContext context)
    {
        if (_planeController == null)
        {
            return;
        }

        var input = context.ReadValue<float>();

        _controlInput = new Vector3(_controlInput.x, input, _controlInput.z);
    }
}