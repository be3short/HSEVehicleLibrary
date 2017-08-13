package edu.ucsc.cross.hse.model.vehicle.pointmass;

import edu.ucsc.cross.hse.model.position.general.Position;
import edu.ucsc.cross.hse.model.position.general.PositionState;

public interface PointMassVehicleControlInput
{

	/*
	 * Orientation angle (in radians) input to the point mass vehicle
	 */
	public Double getOrientationInput(PositionState vehicle_location_state);

	/*
	 * Percentage of maximum planar velocity to apply : range (0,1)
	 */
	public Double getPlanarVelocityInput(PositionState vehicle_location_state);

	/*
	 * Percentage of maximum vertical velocity to apply : range (-1,1)
	 */
	public Double getVerticalVelocityInput(PositionState vehicle_location_state);
}
