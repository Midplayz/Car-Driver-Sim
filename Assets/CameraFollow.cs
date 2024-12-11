using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target; 
    public Vector3 offset = new Vector3(0, 5, -10); 
    public float smoothTime = 0.3f; 
    public float positionSmoothingFactor = 0.95f; 

    private Vector3 velocity = Vector3.zero;
    private Vector3 previousTargetPosition; 

    private void FixedUpdate()
    {
        Vector3 interpolatedTargetPosition = Vector3.Lerp(previousTargetPosition, target.position, positionSmoothingFactor);

        Vector3 desiredPosition = interpolatedTargetPosition + target.TransformDirection(offset);

        transform.position = Vector3.SmoothDamp(transform.position, desiredPosition, ref velocity, smoothTime);

        previousTargetPosition = target.position;

        transform.LookAt(target);
    }
}
