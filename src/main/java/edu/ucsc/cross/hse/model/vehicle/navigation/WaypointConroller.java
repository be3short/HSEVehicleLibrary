package edu.ucsc.cross.hse.model.vehicle.navigation;

import edu.ucsc.cross.hse.core.framework.component.Component;
import edu.ucsc.cross.hse.core.framework.data.Data;
import edu.ucsc.cross.hse.model.position.general.Position;
import edu.ucsc.cross.hse.model.position.general.PositionData;
import edu.ucsc.cross.hse.model.position.general.PositionState;
import java.util.ArrayList;

public abstract class WaypointConroller extends Component
{

	public Data<Boolean> destinationReached;
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
	public WaypointConroller(Double horiz_prox_threshold, Double vert_prox_threshold, Position... waypoints)
	{
		super("Waypoint Controller");
		instantiateElements(horiz_prox_threshold, vert_prox_threshold, waypoints);
	}

	/*
	 * Constructor without thresholds or waypoints defined
	 */
	public WaypointConroller()
	{
		super("Waypoint Controller");
		instantiateElements(0.0, 0.0);
	}

	/*
	 * Instantiates state and data elements
	 */
	private void instantiateElements(Double horiz_prox_threshold, Double vert_prox_threshold, Position... waypoints)
	{
		destinationReached = new Data<Boolean>("Destination Reached", false);
		horizontalProximityThreshold = new Data<Double>("Horizontal Proximity Threshold", horiz_prox_threshold);
		verticalProximityThreshold = new Data<Double>("Vertical Proximity Threshold", vert_prox_threshold);
		pathWaypoints = new Data<ArrayList<Position>>("Waypoint Queue", new ArrayList<Position>());
		currentWaypoint = new Data<Position>("Current Waypoint Index", PositionData.getNullPosition());
		addWaypoints(waypoints);
	}

	/*
	 * Add waypoints to the path
	 */
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
	public void clearWaypoints()
	{
		destinationReached.setValue(false);
		pathWaypoints.getValue().clear();
		currentWaypoint.setValue(PositionData.getNullPosition());
	}

	public boolean destinationReached(PositionState vehicle_location_state)
	{
		boolean destinationReached = false;
		if (pathWaypoints.getValue().size() > 0)
		{
			if (currentWaypoint.getValue().equals(pathWaypoints.getValue().get(pathWaypoints.getValue().size() - 1)))
			{
				destinationReached = currentWaypointReached(vehicle_location_state);

			}
		}
		this.destinationReached.setValue(destinationReached);
		return destinationReached;
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
				destinationReached(vehicle_location_state);
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

}
