package edu.ucsc.cross.hse.model.vehicle.pointmass;

import edu.ucsc.cross.hse.core.framework.annotations.LibraryDefinition;
import edu.ucsc.cross.hse.core.framework.models.HybridSystem;
import edu.ucsc.cross.hse.model.position.general.PositionState;
import edu.ucsc.cross.hse.model.position.general.PositionStateData;
import edu.ucsc.cross.hse.model.vehicle.general.Vehicle;
import edu.ucsc.cross.hse.model.vehicle.navigation.DestinationControl;

public class PointMassVehicleSystem<T> extends Vehicle<T> implements HybridSystem
{

	public PointMassVehicleParameters parameters; // vehicle parameters
	public PointMassVehicleControlInput input; // vehicle controller

	/*
	 * Constructor for a point mass vehicle dynamical system
	 * 
	 * @param state - geographic state of the vehicle
	 * 
	 * @param parameters - point mass parameters of the vehicle
	 * 
	 * @param controller - point mass vehicle control input
	 */
	public PointMassVehicleSystem(PositionState state, PointMassVehicleParameters parameters,
	PointMassVehicleControlInput controller)
	{
		super(state);
		super.component().getLabels().setName("Point Mass Vehicle System");
		T controlType = (T) controller; // check for correct controller
		// this.state = state;
		this.parameters = parameters;
		this.input = controller;
	}

	/*
	 * Determines the derivatives of each state component
	 */
	@Override
	public void flowMap()
	{
		state.getXPositionState().setDerivative(getXVelocity());
		state.getYPositionState().setDerivative(getYVelocity());
		state.getZPositionState().setDerivative(getZVelocity());
	}

	/*
	 * Determines if the vehicle is in the flow set, which occurs when the vehicle is in motion
	 */
	@Override
	public boolean flowSet()
	{
		boolean inMotion = (Math.abs(input.getVerticalVelocityInput(state)) + input.getPlanarVelocityInput(state)) > 0;
		return inMotion;
	}

	/*
	 * Continuous system therefore no jump map
	 */
	@Override
	public void jumpMap()
	{
	}

	/*
	 * Continuous system therefore never in jump set
	 */
	@Override
	public boolean jumpSet()
	{
		return false;
	}

	/*
	 * Computes the planar velocity magnitude of the vehicle
	 */
	public Double getPlanarVelocityMagnitude()
	{
		Double velocity = parameters.getMaximumPlanarVelocity() * input.getPlanarVelocityInput(state);
		return velocity;
	}

	/*
	 * Computes the x velocity of the vehicle
	 */
	public Double getXVelocity()
	{
		Double xVelocity = getPlanarVelocityMagnitude() * Math.cos(input.getOrientationInput(state));
		return xVelocity;
	}

	/*
	 * Computes the y velocity of the vehicle
	 */
	public Double getYVelocity()
	{
		Double xVelocity = getPlanarVelocityMagnitude() * Math.sin(input.getOrientationInput(state));
		return xVelocity;
	}

	/*
	 * Computes the z velocity of the vehicle
	 */
	public Double getZVelocity()
	{
		Double zVelocity = input.getVerticalVelocityInput(state);
		return zVelocity;
	}

	@LibraryDefinition(label = "Simple Point Mass Vehicle System With Empty Path")
	public static PointMassVehicleSystem<DestinationControl> getSimplePointMassVehicleSystem()
	{
		PositionStateData position = new PositionStateData();
		SimplePointMassVehicleParameters parameters = new SimplePointMassVehicleParameters(1.0, 1.0);
		SimplePointMassVehicleNavigationController controller = new SimplePointMassVehicleNavigationController(.1, .1,
		200.0);
		PointMassVehicleSystem<DestinationControl> system = new PointMassVehicleSystem<DestinationControl>(position,
		parameters, controller);
		return system;
	}

	@SuppressWarnings("unchecked")
	@Override
	public T getVehicleController()
	{
		return (T) input;
	}

}
