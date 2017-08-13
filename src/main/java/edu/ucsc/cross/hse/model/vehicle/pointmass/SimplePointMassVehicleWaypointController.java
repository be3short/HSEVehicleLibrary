package edu.ucsc.cross.hse.model.vehicle.pointmass;

import java.util.ArrayList;

import edu.ucsc.cross.hse.core.framework.component.Component;
import edu.ucsc.cross.hse.core.framework.data.Data;
import edu.ucsc.cross.hse.model.position.euclidean.EuclideanPositionData;
import edu.ucsc.cross.hse.model.position.general.Position;
import edu.ucsc.cross.hse.model.position.general.PositionState;
import edu.ucsc.cross.hse.model.vehicle.navigation.WaypointNavigation;

public class SimplePointMassVehicleWaypointController extends Component
implements PointMassVehicleControlInput, WaypointNavigation
{

	/*
	 * Horizontal distance from exact waypoint to consider vehicle arrived.
	 */
	public Data<Double> horizontalProximityThreshold;

	/*
	 * Horizontal distance from exact waypoint to consider vehicle arrived.
	 */
	public Data<Double> verticalProximityThreshold;

	/*
	 * Waypoint that vehicle is currently traveling towards
	 */
	public Data<Position> currentWaypoint;

	/*
	 * List of waypoints that define the path
	 */
	public Data<ArrayList<Position>> pathWaypoints;

	/*
	 * Constructor with thresholds defined and potentially including waypoints
	 */
	public SimplePointMassVehicleWaypointController(Double horiz_prox_threshold, Double vert_prox_threshold,
	Position... waypoints)
	{
		super("Simple Point Mass Vehicle Waypoint Controller");
		instantiateElements(horiz_prox_threshold, vert_prox_threshold, waypoints);
	}

	/*
	 * Constructor without thresholds or waypoints defined
	 */
	public SimplePointMassVehicleWaypointController()
	{
		super("Simple Point Mass Vehicle Waypoint Controller");
		instantiateElements(0.0, 0.0);
	}

	/*
	 * Instantiates state and data elements
	 */
	private void instantiateElements(Double horiz_prox_threshold, Double vert_prox_threshold, Position... waypoints)
	{
		horizontalProximityThreshold = new Data<Double>("Horizontal Proximity Threshold", horiz_prox_threshold);
		verticalProximityThreshold = new Data<Double>("Vertical Proximity Threshold", vert_prox_threshold);
		pathWaypoints = new Data<ArrayList<Position>>("Waypoint Queue", new ArrayList<Position>());
		currentWaypoint = new Data<Position>("Current Waypoint Index", EuclideanPositionData.getNullPosition());
		addWaypoints(waypoints);
	}

	public void checkWaypointLoaded()
	{
		if (currentWaypoint.getValue().isNullPosition())
		{
			if (pathWaypoints.getValue().size() > 0)
			{
				currentWaypoint.setValue(pathWaypoints.getValue().get(0));
			}
		}
	}

	/*
	 * Add waypoints to the path
	 */
	@Override
	public void addWaypoints(Position... waypoints)
	{
		for (Position waypoint : waypoints)
		{
			pathWaypoints.getValue().add(waypoint);
		}
	}

	/*
	 * Clear waypoints
	 */
	@Override
	public void clearWaypoints()
	{
		pathWaypoints.getValue().clear();
		currentWaypoint.setValue(EuclideanPositionData.getNullPosition());
	}

	@Override
	public Double getOrientationInput(PositionState vehicle_location_state)
	{
		updateWaypoint(vehicle_location_state);
		return computeTargetAngle(vehicle_location_state);
	}

	@Override
	public Double getPlanarVelocityInput(PositionState vehicle_location_state)
	{
		updateWaypoint(vehicle_location_state);
		return computePlanarVelocityInput(vehicle_location_state);
	}

	@Override
	public Double getVerticalVelocityInput(PositionState vehicle_location_state)
	{
		updateWaypoint(vehicle_location_state);
		return computeVerticalVelocityInput(vehicle_location_state);
	}

	public void updateWaypoint(PositionState vehicle_location_state)
	{
		checkWaypointLoaded();
		if (currentWaypoint.getValue() != null)
		{
			if (currentWaypointReached(vehicle_location_state))
			{
				Integer waypointIndex = pathWaypoints.getValue().indexOf(currentWaypoint.getValue());
				if (waypointIndex >= 0 && (waypointIndex < pathWaypoints.getValue().size() - 1))
				{
					System.out.println("Waypoint Reached");
					currentWaypoint.setValue(pathWaypoints.getValue().get(++waypointIndex));
				}
			}
		}
	}

	public Double computeVerticalVelocityInput(PositionState vehicle_location_state)
	{
		Double velocity = 0.0;
		if (currentWaypoint.getValue() != null)
		{
			if (!withinVerticalProximity(vehicle_location_state))
			{
				velocity = 1.0 * Math.signum(
				currentWaypoint.getValue().getZPosition() - vehicle_location_state.getZPositionState().getValue());
			}
		}
		return velocity;
	}

	public Double computePlanarVelocityInput(PositionState vehicle_location_state)
	{
		Double velocity = 0.0;
		if (currentWaypoint.getValue() != null)
		{
			if (!withinHorizontalProximity(vehicle_location_state))
			{
				velocity = 1.0;
			}
		}
		return velocity;
	}

	public boolean withinVerticalProximity(PositionState vehicle_location_state)
	{
		Double verticalDistance = currentWaypoint.getValue().getZPosition()
		- vehicle_location_state.getZPositionState().getValue();
		boolean withinVerticalProximity = Math.abs(verticalDistance) < verticalProximityThreshold.getValue();
		return withinVerticalProximity;
	}

	public boolean withinHorizontalProximity(PositionState vehicle_location_state)
	{
		Double horizontalDistance = Math.sqrt(
		Math.pow((currentWaypoint.getValue().getXPosition() - vehicle_location_state.getXPositionState().getValue()), 2)
		+ Math.pow((currentWaypoint.getValue().getYPosition() - vehicle_location_state.getYPositionState().getValue()),
		2));
		boolean withinVerticalProximity = Math.abs(horizontalDistance) < horizontalProximityThreshold.getValue();
		return withinVerticalProximity;
	}

	public boolean currentWaypointReached(PositionState vehicle_location_state)
	{
		boolean reached = withinVerticalProximity(vehicle_location_state)
		&& withinHorizontalProximity(vehicle_location_state);
		return reached;
	}

	public Double computeTargetAngle(PositionState vehicle_location_state)
	{
		Double angle = 0.0;
		if (currentWaypoint.getValue() != null)
		{
			angle = Math
			.atan2(currentWaypoint.getValue().getYPosition() - vehicle_location_state.getYPositionState().getValue(),
			currentWaypoint.getValue().getXPosition() - vehicle_location_state.getXPositionState().getValue());
		}
		return angle;
	}
}
