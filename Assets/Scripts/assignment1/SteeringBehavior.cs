// Collaboration by: Sean Massa & Zuhair Jafri

using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class SteeringBehavior : MonoBehaviour
{
    public Vector3 target;
    public KinematicBehavior kinematic;
    public List<Vector3> path;
    public TextMeshProUGUI label;

    // how close we get to the target
    public float arrivalThreshold = 0.5f;

    // Minimum speed for the car
    public float minSpeed = 4f;
    public float maxSpeed = 10f;
    // Smooth turning parameters
    public float rotationSpeed = 10f;  // Rotation speed (degrees per second)
    private float turnSmoothVelocity;

    private bool usingDirectTarget = false;

    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        target = transform.position;
        path = null;
        EventBus.OnSetMap += SetMap;
    }

    void Update()
    {
        // If there's a path, follow it, else move directly to target
        if (path != null && path.Count > 0)
        {
            FollowPath();
        }
        else
        {
            // Direct movement to target when path is empty
            MoveToTarget();
        }
    }

    // Direct movement towards the target (when there's no path)
    private void MoveToTarget()
    {
        Vector3 toTarget = target - transform.position;
        float distance = toTarget.magnitude;

        if (label != null)
        {
            label.text = "Distance: " + distance.ToString("F2");
        }

        // If close to target, stop
        if (distance < arrivalThreshold)
        {
            kinematic.SetDesiredSpeed(0);
            kinematic.SetDesiredRotationalVelocity(0);
            return;
        }

        // Steering logic to face the target
        Vector3 direction = toTarget.normalized;
        float angleToTarget = Vector3.SignedAngle(transform.forward, direction, Vector3.up);
        float rotationStrength = Mathf.Clamp(angleToTarget / 45f, -1f, 1f);
        kinematic.SetDesiredRotationalVelocity(rotationStrength);

        // Adjust speed based on distance and alignment
        float alignmentFactor = Mathf.Clamp01(1f - Mathf.Abs(angleToTarget) / 45f);
        float distanceFactor = Mathf.Clamp01(distance / 5f); // Slow down when close to target
        float desiredSpeed = maxSpeed * alignmentFactor * distanceFactor;
        desiredSpeed = Mathf.Max(minSpeed, desiredSpeed);

        kinematic.SetDesiredSpeed(desiredSpeed);
    }

    private void FollowPath()
    {
        // Determine the current movement target (first path point or target if no path)
        Vector3 currentTarget = (path != null && path.Count > 0) ? path[0] : target;
        Vector3 toTarget = currentTarget - transform.position;
        float distance = toTarget.magnitude;

        if (label != null)
        {
            label.text = "Distance: " + distance.ToString("F2");
        }

        // If close to target, stop or move to the next path point
        if (distance < arrivalThreshold)
        {
            if (path != null && path.Count > 0)
            {
                path.RemoveAt(0); // Move to the next waypoint
            }
            else
            {
                // Stop at final destination
                kinematic.SetDesiredSpeed(0);
                kinematic.SetDesiredRotationalVelocity(0);
            }
            return;
        }

        // Calculate direction to the target
        Vector3 direction = toTarget.normalized;

        // Calculate angle to target from the car's forward direction
        float angleToTarget = Vector3.SignedAngle(transform.forward, direction, Vector3.up);

        // Smooth rotation towards the target
        float rotationVelocity = Mathf.Sign(angleToTarget) * rotationSpeed;
        float smoothRotation = Mathf.SmoothDampAngle(transform.eulerAngles.y, transform.eulerAngles.y + angleToTarget, ref turnSmoothVelocity, 0.1f);
        kinematic.SetDesiredRotationalVelocity(smoothRotation);

        // Calculate speed based on distance and angle
        float alignmentFactor = Mathf.Clamp01(1f - Mathf.Abs(angleToTarget) / 45f); // Slow down more if turning
        float distanceFactor = Mathf.Clamp01(distance / 5f);  // Slow down as we get closer
        float desiredSpeed = maxSpeed * alignmentFactor * distanceFactor;
        desiredSpeed = Mathf.Max(minSpeed, desiredSpeed);

        // Set desired speed
        kinematic.SetDesiredSpeed(desiredSpeed);
    }
    public void SetTarget(Vector3 target)
    {
        this.target = target;
        EventBus.ShowTarget(target);
    }

    public void SetPath(List<Vector3> path)
    {
        this.path = path;
    }

    public void SetMap(List<Wall> outline)
    {
        this.path = null;
        this.target = transform.position;
    }
}
