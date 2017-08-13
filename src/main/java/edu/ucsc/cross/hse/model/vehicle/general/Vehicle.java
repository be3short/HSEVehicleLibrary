package edu.ucsc.cross.hse.model.vehicle.general;

import edu.ucsc.cross.hse.core.framework.component.Component;
import edu.ucsc.cross.hse.core.framework.data.State;
import edu.ucsc.cross.hse.model.position.general.Position;
import edu.ucsc.cross.hse.model.position.general.PositionState;

public abstract class Vehicle<T> extends Component implements Position, ControlType<T>
{

	public PositionState state;

	public Vehicle(PositionState state)
	{
		this.state = state;
	}

	/*
	 * Get the value of the x state component
	 */
	@Override
	public Double getXPosition()
	{
		// TODO Auto-generated method stub
		return state.getXPositionState().getValue();
	}

	/*
	 * Get the value of the y state component
	 */
	@Override
	public Double getYPosition()
	{
		// TODO Auto-generated method stub
		return state.getXPositionState().getValue();
	}

	/*
	 * Get the value of the z state component
	 */
	@Override
	public Double getZPosition()
	{
		// TODO Auto-generated method stub
		return state.getXPositionState().getValue();
	}

	@Override
	public boolean isNullPosition()
	{
		// TODO Auto-generated method stub
		return state == null;
	}
}
