package com.ur.thph.modbus_urcap.impl;

import org.osgi.framework.BundleActivator;
import org.osgi.framework.BundleContext;

import com.ur.urcap.api.contribution.DaemonService;
import com.ur.urcap.api.contribution.driver.gripper.GripperContribution;



/**
 * Hello world activator for the OSGi bundle URCAPS contribution
 *
 */
public class Activator implements BundleActivator {
	@Override
	public void start(BundleContext bundleContext) throws Exception {
		System.out.println("Activator says Modbus Service!");
		
		ModbusDaemonService modbusDaemonService = new ModbusDaemonService();
		
		
		bundleContext.registerService(DaemonService.class, modbusDaemonService, null);
		bundleContext.registerService(GripperContribution.class, new ModbusVGP20Gripper(modbusDaemonService), null);
	}

	@Override
	public void stop(BundleContext bundleContext) throws Exception {
		System.out.println("Activator says Modbus ServiceS!");
	}
}

