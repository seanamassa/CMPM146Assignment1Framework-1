// Collaboration by: Sean Massa & Zuhair Jafri

using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class SteeringBehavior : MonoBehaviour
{
    // --- Original Member Variables ---
    public Vector3 target;
    public KinematicBehavior kinematic;
    public List<Vector3> path;
    public TextMeshProUGUI label;

    // --- Steering Variables ---
    public float targetRadius = 0.5f; // Radius around target to consider arrived
    public float slowRadius = 5.0f;   // Radius to start slowing down for target/waypoint
    public float waypointRadius = 1.0f; // Radius around waypoint to consider it reached
    public float rotationSlowRadiusDeg = 15.0f; // Start slowing rotation when angle difference is less than this
    public float rotationTargetRadiusDeg = 2.0f; // Consider rotation complete when angle difference is less than this

    private int currentPathIndex = 0; // Index for the current waypoint in the path

    // --- ADD THIS FLAG ---
    private bool hasActiveTarget = false; // Track if a target/path has been set by user
    // --------------------


    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        if (kinematic == null)
        {
            Debug.LogError("SteeringBehavior requires a KinematicBehavior component on the same GameObject.", this);
            this.enabled = false;
            return;
        }
        target = transform.position; // Keep this initialization
        path = null;
        hasActiveTarget = false; // Initialize flag
        EventBus.OnSetMap += SetMap;
    }

    void Update()
    {
        if (kinematic == null) return;

        // --- EXIT EARLY IF NO ACTIVE TARGET ---
        if (!hasActiveTarget)
        {
            StopMovement();
            if (label != null) label.text = "Waiting for Target...";
            return;
        }
        // --------------------------------------


        Vector3 currentTarget;
        bool isFollowingPath = (path != null && path.Count > 0);
        bool isFinalDestination = false;

        // Determine currentTarget and isFinalDestination
        if (isFollowingPath)
        {
            if (currentPathIndex >= path.Count)
            {
                StopMovement();
                if (label != null) label.text = "Path Complete";
                // hasActiveTarget = false; // Optional: Idle after path
                return;
            }

            currentTarget = path[currentPathIndex];
            isFinalDestination = (currentPathIndex == path.Count - 1);

            float distanceToWaypoint = Vector3.Distance(transform.position, currentTarget);
            if (label != null) label.text = $"To Waypoint {currentPathIndex}: {distanceToWaypoint:F1}m / Final? {isFinalDestination}";

            // Waypoint Advancement
            if (distanceToWaypoint < waypointRadius)
            {
                if (isFinalDestination)
                {
                    // Let the normal stop check run below
                }
                else
                {
                    currentPathIndex++;
                    if (currentPathIndex >= path.Count) {
                        // No more waypoints, let the stopping condition below handle it
                    } else {
                        currentTarget = path[currentPathIndex];
                        isFinalDestination = (currentPathIndex == path.Count - 1);
                    }
                }
            }
        }
        else // Single target mode (and hasActiveTarget is true)
        {
            currentTarget = this.target;
            isFinalDestination = true;
            float distanceToTarget = Vector3.Distance(transform.position, currentTarget);
            if (label != null) label.text = $"To Target: {distanceToTarget:F1}m";
        }

        // --- Check for immediate stop condition FIRST (Requires active target AND final destination) ---
        float distance = Vector3.Distance(transform.position, currentTarget);

        // --- ADD/KEEP THIS LOG ---
        //Debug.Log($"Stop Check Values >> Active: {hasActiveTarget}, Final: {isFinalDestination}, Dist: {distance:F2}, Radius: {targetRadius}");

        // Strong stop condition block
        if (hasActiveTarget && isFinalDestination && distance < targetRadius)
        {
            Debug.Log($"Stopping movement >> Distance: {distance:F2}, TargetRadius: {targetRadius}");
            StopMovement();
            if (label != null)
            {
                label.text = isFollowingPath ? "Path Complete" : "Target Reached";
            }
            return; // Exit early so we don’t compute velocity/rotation
        }

         Vector3 direction = currentTarget - transform.position;
         float desiredSpeed;
         if (distance < slowRadius)
         {
             // Ensure slowRadius > targetRadius for this calculation!
             if (slowRadius <= targetRadius) {
                  //Debug.LogWarning("slowRadius should be greater than targetRadius for arrival behavior!");
                  desiredSpeed = (distance < targetRadius) ? 0 : kinematic.max_speed; // Basic handling if radii invalid
             } else {
                 float speedFactor = Mathf.Clamp01((distance - targetRadius) / (slowRadius - targetRadius));
                 desiredSpeed = kinematic.max_speed * speedFactor;
             }
         } else {
             desiredSpeed = kinematic.max_speed;
         }
        desiredSpeed = Mathf.Max(0, desiredSpeed);
        kinematic.SetDesiredSpeed(desiredSpeed);

        // Rotation calculation
        float desiredRotationalVelocity = 0;
        if (distance > targetRadius && desiredSpeed > 0.01f) {
            float targetAngle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;
            float currentAngle = transform.eulerAngles.y;
            float angleDifference = Mathf.DeltaAngle(currentAngle, targetAngle);
            if (Mathf.Abs(angleDifference) < rotationTargetRadiusDeg) { desiredRotationalVelocity = 0; }
            else if (Mathf.Abs(angleDifference) < rotationSlowRadiusDeg) { desiredRotationalVelocity = kinematic.max_rotational_velocity * (angleDifference / rotationSlowRadiusDeg); }
            else { desiredRotationalVelocity = kinematic.max_rotational_velocity * Mathf.Sign(angleDifference); }
            if (!kinematic.holonomic && kinematic.speed < 1.0f) { desiredRotationalVelocity *= Mathf.Clamp01(kinematic.speed / 1.0f); }
        }
        kinematic.SetDesiredRotationalVelocity(desiredRotationalVelocity);

    } // End of Update()


    void StopMovement()
    {
        // Debugging StopMovement
        //Debug.LogError("!!! StopMovement() CALLED !!!");

        if (kinematic != null)
        {
            kinematic.SetDesiredSpeed(0);
            kinematic.SetDesiredRotationalVelocity(0);
            kinematic.speed = 0;
            kinematic.rotational_velocity = 0;
        }
    }

    public void SetTarget(Vector3 target)
    {
        this.path = null;
        this.target = target;
        this.currentPathIndex = 0; // Reset path index just in case
        this.hasActiveTarget = true; // << SET FLAG HERE
        EventBus.ShowTarget(target);
         if (label != null) label.text = "Target Set";
         // Remove the immediate Update() call here, let the normal Update loop handle it
         // if (kinematic != null) { Update(); }
    }

    public void SetPath(List<Vector3> path)
    {
        if (path == null || path.Count == 0)
        {
            this.path = null;
            this.target = transform.position; // Reset target to current position
            this.hasActiveTarget = false; // << CLEAR FLAG HERE
            if (label != null) label.text = "Path Cleared";
            if (kinematic != null) {
                 StopMovement(); // Ensure stop when path cleared
            }
        }
        else
        {
            this.path = new List<Vector3>(path);
            this.currentPathIndex = 0;
            this.target = this.path[currentPathIndex]; // Set target for initial calculations
            this.hasActiveTarget = true; // << SET FLAG HERE
            // EventBus.ShowPath(path); // Still commented out
            if (label != null) label.text = "Path Set";
        }
    }

    public void SetMap(List<Wall> outline)
    {
        // Reset state when map changes
        this.path = null;
        this.target = transform.position;
        this.currentPathIndex = 0;
        this.hasActiveTarget = false; // << CLEAR FLAG HERE too on map reset
        if (label != null) label.text = "Map Reset";
        if (kinematic != null)
        {
            StopMovement(); // Ensure stop on map reset
        }
    }

    void OnDestroy()
    {
        EventBus.OnSetMap -= SetMap;
    }
}