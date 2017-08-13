package edu.ucsc.cross.hse.models.app;

import edu.ucsc.cross.hse.core.framework.data.Data;
import edu.ucsc.cross.hse.core.framework.models.HybridSystem;
import edu.ucsc.cross.hse.model.position.general.PositionData;
import edu.ucsc.cross.hse.model.vehicle.navigation.SimpleNavigationController;
import edu.ucsc.cross.hse.model.vehicle.pointmass.SimplePointMassVehicleNavigationController;

public class RandomizedSimplePointMassVehicleNavigationController extends SimplePointMassVehicleNavigationController
implements HybridSystem
{

	Data<Double> xPosRange;
	Data<Double> yPosRange;
	Data<Double> zPosRange;

	public RandomizedSimplePointMassVehicleNavigationController()
	{
		super(.1, .1, 100.0);
		xPosRange = new Data<Double>("X Position Generation Range", 10.0);
		yPosRange = new Data<Double>("Y Position Generation Range", 10.0);
		zPosRange = new Data<Double>("Z Position Generation Range", 10.0);
	}

	@Override
	public void flowMap()
	{
		if (super.destinationReached.getValue())
		{
			super.updateDestination(generateNewDestination());
		}
	}

	public PositionData generateNewDestination()
	{
		Double x = Math.random() * xPosRange.getValue() * Math.signum(0 - Math.ceil(Math.random()));
		Double y = Math.random() * yPosRange.getValue() * Math.signum(0 - Math.ceil(Math.random()));
		Double z = Math.random() * zPosRange.getValue();
		return new PositionData(x, y, z);
	}

	@Override
	public boolean flowSet()
	{
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	public void jumpMap()
	{
		// TODO Auto-generated method stub

	}

	@Override
	public boolean jumpSet()
	{
		// TODO Auto-generated method stub
		return false;
	}

}
