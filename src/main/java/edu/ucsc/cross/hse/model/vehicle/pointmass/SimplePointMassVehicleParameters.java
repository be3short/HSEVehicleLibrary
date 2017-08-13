package edu.ucsc.cross.hse.model.vehicle.pointmass;

import edu.ucsc.cross.hse.core.framework.component.Component;
import edu.ucsc.cross.hse.core.framework.data.Data;

public class SimplePointMassVehicleParameters extends Component implements PointMassVehicleParameters
{

	public Data<Double> maximumPlanarVelocity;
	public Data<Double> maximumVerticalVelocity;

	public SimplePointMassVehicleParameters(Double max_planar_velocity, Double max_vertical_velocity)
	{
		instantiateElements(max_planar_velocity, max_vertical_velocity);
	}

	private void instantiateElements(Double max_planar_velocity, Double max_vertical_velocity)
	{
		maximumPlanarVelocity = new Data<Double>("Maximum Planar Velocity", max_planar_velocity);
		maximumVerticalVelocity = new Data<Double>("Maximum Vertical Velocity", max_vertical_velocity);
	}

	@Override
	public Double getMaximumPlanarVelocity()
	{
		return maximumPlanarVelocity.getValue();
	}

	@Override
	public Double getMaximumVerticalVelocity()
	{
		return maximumVerticalVelocity.getValue();
	}
}
