using UnityEngine;

public class CarController : MonoBehaviour
{
    [Header("Wheel Colliders")]
    public WheelCollider frontLeftCollider;
    public WheelCollider frontRightCollider;
    public WheelCollider rearLeftCollider;
    public WheelCollider rearRightCollider;

    [Header("Wheel Meshes")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;

    [Header("Car Settings")]
    public float maxMotorTorque = 1500f; 
    public float maxSteeringAngle = 30f; 
    public float brakeForce = 3000f; 
    public float eBrakeForce = 5000f; 
    public float driftStiffness = 0.5f; 

    [Header("Visual Effects")]
    public ParticleSystem skidSmoke; 

    private float motorInput;
    private float steeringInput;
    private float brakeInput;
    private bool isDrifting;
    private Rigidbody rb;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        GetInput();
        HandleMotor();
        HandleSteering();
        HandleBrakes();
        ApplyDrifting();
        UpdateWheelPoses();

        ApplyAntiRollBar(frontLeftCollider, frontRightCollider);
        ApplyAntiRollBar(rearLeftCollider, rearRightCollider);
    }

    private void GetInput()
    {
        motorInput = Input.GetAxis("Vertical"); 
        steeringInput = Input.GetAxis("Horizontal");
        brakeInput = Input.GetKey(KeyCode.Space) ? brakeForce : 0f; 
        isDrifting = Input.GetKey(KeyCode.LeftShift); 
    }

    private void HandleMotor()
    {
        rearLeftCollider.motorTorque = motorInput * maxMotorTorque;
        rearRightCollider.motorTorque = motorInput * maxMotorTorque;

        if (brakeInput > 0)
        {
            rearLeftCollider.motorTorque = 0;
            rearRightCollider.motorTorque = 0;
        }
    }

    private void HandleSteering()
    {
        float steering = steeringInput * maxSteeringAngle;
        frontLeftCollider.steerAngle = steering;
        frontRightCollider.steerAngle = steering;
    }

    private void HandleBrakes()
    {
        frontLeftCollider.brakeTorque = brakeInput;
        frontRightCollider.brakeTorque = brakeInput;
        rearLeftCollider.brakeTorque = brakeInput;
        rearRightCollider.brakeTorque = brakeInput;

        if (Input.GetKey(KeyCode.Space))
        {
            rearLeftCollider.brakeTorque = eBrakeForce;
            rearRightCollider.brakeTorque = eBrakeForce;
        }
    }

    private void ApplyDrifting()
    {
        if (isDrifting)
        {
            WheelFrictionCurve driftCurve = rearLeftCollider.sidewaysFriction;
            driftCurve.stiffness = driftStiffness;
            rearLeftCollider.sidewaysFriction = driftCurve;
            rearRightCollider.sidewaysFriction = driftCurve;

            if (skidSmoke && !skidSmoke.isPlaying)
            {
                skidSmoke.Play();
            }
        }
        else
        {
            WheelFrictionCurve normalCurve = rearLeftCollider.sidewaysFriction;
            normalCurve.stiffness = 1f;
            rearLeftCollider.sidewaysFriction = normalCurve;
            rearRightCollider.sidewaysFriction = normalCurve;

            if (skidSmoke && skidSmoke.isPlaying)
            {
                skidSmoke.Stop();
            }
        }
    }

    private void UpdateWheelPoses()
    {
        UpdateWheelPose(frontLeftCollider, frontLeftMesh, true); 
        UpdateWheelPose(frontRightCollider, frontRightMesh, true); 
        UpdateWheelPose(rearLeftCollider, rearLeftMesh, false); 
        UpdateWheelPose(rearRightCollider, rearRightMesh, false); 
    }

    private void UpdateWheelPose(WheelCollider collider, Transform mesh, bool isSteeringWheel)
    {
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        mesh.position = position;

        Quaternion rollRotation = Quaternion.Euler(rotation.eulerAngles.x, 0, 0);

        if (isSteeringWheel)
        {
            Quaternion steerRotation = Quaternion.Euler(0, collider.steerAngle, 0);
            mesh.rotation = collider.transform.rotation * steerRotation * rollRotation;
        }
        else
        {
            mesh.rotation = collider.transform.rotation * rollRotation;
        }
    }


    private void ApplyAntiRollBar(WheelCollider leftWheel, WheelCollider rightWheel)
    {
        WheelHit hit;
        float travelLeft = 1f;
        float travelRight = 1f;

        if (leftWheel.GetGroundHit(out hit))
        {
            travelLeft = (-leftWheel.transform.InverseTransformPoint(hit.point).y - leftWheel.radius) / leftWheel.suspensionDistance;
        }
        if (rightWheel.GetGroundHit(out hit))
        {
            travelRight = (-rightWheel.transform.InverseTransformPoint(hit.point).y - rightWheel.radius) / rightWheel.suspensionDistance;
        }

        float antiRollForce = (travelLeft - travelRight) * 5000f;

        if (leftWheel.isGrounded)
        {
            rb.AddForceAtPosition(leftWheel.transform.up * -antiRollForce, leftWheel.transform.position);
        }
        if (rightWheel.isGrounded)
        {
            rb.AddForceAtPosition(rightWheel.transform.up * antiRollForce, rightWheel.transform.position);
        }
    }
}
