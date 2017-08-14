package edu.ucsc.cross.hse.model.vehicle.pointmass;

import java.util.ArrayList;

import edu.ucsc.cross.hse.core.framework.component.Component;
import edu.ucsc.cross.hse.core.framework.data.Data;
import edu.ucsc.cross.hse.model.position.general.Position;
import edu.ucsc.cross.hse.model.position.general.PositionData;
import edu.ucsc.cross.hse.model.position.general.PositionState;
import edu.ucsc.cross.hse.model.vehicle.navigation.DestinationControl;
import edu.ucsc.cross.hse.model.vehicle.navigation.SimpleNavigationController;
import edu.ucsc.cross.hse.model.vehicle.navigation.WaypointConroller;

public class SimplePointMassVehicleNavigationController extends SimpleNavigationController
implements PointMassVehicleControlInput
{

	/*
	 * Constructor with thresholds defined and potentially including waypoints
	 */
	public SimplePointMassVehicleNavigationController(Double travel_height, Double horiz_prox_threshold,
	Double vert_prox_threshold)
	{
		super(travel_height, horiz_prox_threshold, vert_prox_threshold);
	}

	/*
	 * Constructor without thresholds or waypoints defined
	 */
	public SimplePointMassVehicleNavigationController()
	{
		super();
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

	@Override
	public boolean inMotion()
	{
		// TODO Auto-generated method stub
		return super.pathWaypoints.setValue().size() > 0;
	}

}
