package com.ur.thph.modbus_urcap.impl;

import com.ur.urcap.api.contribution.DaemonContribution;
import com.ur.urcap.api.contribution.driver.general.tcp.TCPConfiguration;
import com.ur.urcap.api.contribution.driver.general.userinput.CustomUserInputConfiguration;
import com.ur.urcap.api.contribution.driver.general.userinput.ValueChangedListener;
import com.ur.urcap.api.contribution.driver.general.userinput.selectableinput.BooleanUserInput;

import com.ur.urcap.api.contribution.driver.gripper.ContributionConfiguration;
import com.ur.urcap.api.contribution.driver.gripper.GripActionParameters;
import com.ur.urcap.api.contribution.driver.gripper.GripperAPIProvider;
import com.ur.urcap.api.contribution.driver.gripper.GripperConfiguration;
import com.ur.urcap.api.contribution.driver.gripper.GripperContribution;
import com.ur.urcap.api.contribution.driver.gripper.ReleaseActionParameters;
import com.ur.urcap.api.contribution.driver.gripper.SystemConfiguration;
import com.ur.urcap.api.contribution.driver.gripper.capability.GripVacuumCapability;
import com.ur.urcap.api.contribution.driver.gripper.capability.GripperCapabilities;
import com.ur.urcap.api.contribution.driver.gripper.capability.multigripper.GripperList;
import com.ur.urcap.api.contribution.driver.gripper.capability.multigripper.GripperListBuilder;
import com.ur.urcap.api.contribution.driver.gripper.capability.multigripper.GripperListProvider;

import com.ur.urcap.api.domain.program.nodes.contributable.device.gripper.configuration.SelectableGripper;
import com.ur.urcap.api.domain.resource.ControllableResourceModel;
import com.ur.urcap.api.domain.robot.RobotModel.RobotType;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.value.PoseFactory;
import com.ur.urcap.api.domain.value.simple.Angle;
import com.ur.urcap.api.domain.value.simple.Length;
import com.ur.urcap.api.domain.value.simple.Pressure;

import javax.swing.ImageIcon;
import java.util.Locale;
import java.util.Timer;


public class ModbusVGP20Gripper implements GripperContribution {

	private static final String GRIPPER_TITLE = "Dual Zone Gripper";

	private static final String ZONE_A_NAME = "Zone A";
	private static final String ZONE_B_NAME = "Zone B";
	private static final String ZONE_AB_NAME = "Zone A+B";

	private static final String ZONE_A_TCP_NAME = "Zone_A";
	private static final String ZONE_B_TCP_NAME = "Zone_B";
	private static final String ZONE_AB_TCP_NAME = "Zone_AB";

	// Ids must remain constant over time and versions of the Gripper URCap, since they are used for persistence and
	// can be used by other URCaps for configuring Gripper program nodes.
	private static final String ZONE_A_ID = "ZoneA_id";
	private static final String ZONE_B_ID = "ZoneB_id";
	private static final String ZONE_AB_ID = "ZoneAB_id";

	private static final String FRAGILE_HANDLING_LABEL = "Use Fragile Handling";
	private static final String FRAGILE_HANDLING_ID = "fragile_handling_id";

	private SelectableGripper zoneAGripper;
	private SelectableGripper zoneBGripper;
	private SelectableGripper zoneABGripper;
	private GripVacuumCapability gripVacuumCapability;

	private BooleanUserInput fragileHandlingInput;
	
	/*
	 * VGP20 - Hackathon Stuff
	 */
	
	private Timer uiTimer;
	private boolean pauseTimer;
	private final ModbusDaemonService modbusDaemonService;
	private ModbusDaemonInterface modbusDaemonInterface;
	
	private static final int PORT = 50408;						// Port Changed from 40408 -> to 50408
	private static final String HOST = "127.0.0.1";
	
	private static final String SCRIPT_FILE_PATH = "/script/modbus.script";
	private ScriptHandler handler;
	
	private final String DAEMON_STATUS_LABEL = "daemonStatusLabel";
	private final String DAEMON_STATUS_ID = "startDaemonCheckBox";
	
	private RobotType robotType;
	
	private ControllableResourceModel resourceModel;
	private ToolIOController toolIOController ;
	// TODO:  add ToolIO Control 
	
	/*
	 * 
	 */
	

	public ModbusVGP20Gripper(ModbusDaemonService modbusDaemonService) {
		this.modbusDaemonService = modbusDaemonService;
		this.modbusDaemonInterface = new ModbusDaemonInterface(HOST, PORT);
		
		
		
		/*this.handler = new ScriptHandler(this.api); 
		 * 
		 * API Connection is missing
		 * The needed api functionality -> Contributing to the function expression editor
		 * is not available
		 *
		 *this.handler.addFunctionModels(SCRIPT_FILE_PATH);
		 *
		 */
		
		applyDesiredDaemonStatus();
	}

	
	
	private void applyDesiredDaemonStatus() {
		new Thread(new Runnable() {
			@Override
			public void run() {
				if (isDaemonEnabled().booleanValue()) {
					// Download the daemon settings to the daemon process on initial start for
					// real-time preview purposes
					System.out.println("Starting daemon");
					try {
						
						awaitDaemonRunning(5000);
						boolean test = modbusDaemonInterface.isReachable();
						if (test) {
							System.out.println("Daemon is running\nDaemon Status: " + getDaemonState());
						} else {
							System.out.println("Daemon is not running");
						}
					} catch (Exception e) {
						System.err.println("Could not reach the daemon process.\nError Message: " + e);
					} 
				} else {
					System.out.println("  DAEMON WILL BE STOPPED !! ");
					modbusDaemonService.getDaemon().stop();
				}
			}
		}).start();
	}
	
	private void awaitDaemonRunning(long timeOutMilliSeconds) throws InterruptedException {
		this.modbusDaemonService.getDaemon().start();
		long endTime = System.nanoTime() + timeOutMilliSeconds * 1000L * 1000L;
		while (System.nanoTime() < endTime
				&& (this.modbusDaemonService.getDaemon().getState() != DaemonContribution.State.RUNNING
						|| !modbusDaemonInterface.isReachable())) {

			Thread.sleep(100);

		}
	}
	
	private DaemonContribution.State getDaemonState() {
		return this.modbusDaemonService.getDaemon().getState();

	}
	
	private Boolean isDaemonEnabled() {
		return true;
	}
		
	
	@Override
	public String getTitle(Locale locale) {
		return GRIPPER_TITLE;
	}

	@Override
	public void configureContribution(ContributionConfiguration configuration) {
		configuration.setLogo(new ImageIcon(getClass().getResource("/logo/logo.png")));
		
	}

	@Override
	public void configureGripper(GripperConfiguration gripperConfiguration, GripperAPIProvider gripperAPIProvider) {
		GripperCapabilities capabilities = gripperConfiguration.getGripperCapabilities();
		
		capabilities.registerMultiGripperCapability(new GripperListProvider() {
			@Override
			public GripperList getGripperList(GripperListBuilder gripperListBuilder, Locale locale) {
				zoneAGripper = gripperListBuilder.createGripper(ZONE_A_ID, ZONE_A_NAME, true);
				zoneBGripper = gripperListBuilder.createGripper(ZONE_B_ID, ZONE_B_NAME, true);
				zoneABGripper = gripperListBuilder.createGripper(ZONE_AB_ID, ZONE_AB_NAME, true);

				return gripperListBuilder.buildList();
			}
		});

		gripVacuumCapability = capabilities.registerGrippingVacuumCapability(0, 100, 70, Pressure.Unit.KPA);
		
		
	}

	@Override
	public void configureInstallation(CustomUserInputConfiguration configurationUIBuilder,
									  SystemConfiguration systemConfiguration,
									  TCPConfiguration tcpConfiguration,
									  GripperAPIProvider gripperAPIProvider) {
		configureGripperTCPs(systemConfiguration, gripperAPIProvider);
		customizeInstallationScreen(configurationUIBuilder);
		
		configureToolIOResourceModel(systemConfiguration, gripperAPIProvider);
		
		
		
	}
	
	private void configureToolIOResourceModel(SystemConfiguration systemConfiguration, GripperAPIProvider gripperAPIProvider ) {
		this.resourceModel = systemConfiguration.getControllableResourceModel();
		this.toolIOController = new ToolIOController(resourceModel, gripperAPIProvider.getSystemAPI().getCapabilityManager());
		this.resourceModel.requestControl(toolIOController);		
	}

	private void readRobotType(GripperAPIProvider gripperAPIProvider) {
		this.robotType = gripperAPIProvider.getSystemAPI().getRobotModel().getRobotType();
	}
	
	public RobotType getRobotType() {
		return this.robotType;
	}
	
	private void configureGripperTCPs(SystemConfiguration systemConfiguration, GripperAPIProvider gripperAPIProvider) {
		PoseFactory poseFactory = gripperAPIProvider.getPoseFactory();

		TCPConfiguration zoneATCPConfiguration = systemConfiguration.getTCPConfiguration(zoneAGripper);
		zoneATCPConfiguration.setTCP(ZONE_A_TCP_NAME, poseFactory.createPose(75, 0, 50, 0, 0, 0, Length.Unit.MM, Angle.Unit.DEG));

		TCPConfiguration zoneBTCPConfiguration = systemConfiguration.getTCPConfiguration(zoneBGripper);
		zoneBTCPConfiguration.setTCP(ZONE_B_TCP_NAME, poseFactory.createPose(-75, 0, 50, 0, 0, 0, Length.Unit.MM, Angle.Unit.DEG));

		TCPConfiguration zoneABTCPConfiguration = systemConfiguration.getTCPConfiguration(zoneABGripper);
		zoneABTCPConfiguration.setTCP(ZONE_AB_TCP_NAME, poseFactory.createPose(0, 0, 50, 0, 0, 0, Length.Unit.MM, Angle.Unit.DEG));
	}

	private void customizeInstallationScreen(CustomUserInputConfiguration configurationUIBuilder) {
		fragileHandlingInput = configurationUIBuilder.registerBooleanInput(FRAGILE_HANDLING_ID, FRAGILE_HANDLING_LABEL, false);
		fragileHandlingInput.setValueChangedListener(new ValueChangedListener<Boolean>() {
			@Override
			public void onValueChanged(Boolean useFragileHandling) {
				updateVacuumCapability(useFragileHandling);
			}
		});
		configurationUIBuilder.addText("The Daemon is ",   getDaemonState() +" Perfect!");
		configurationUIBuilder.registerBooleanInput(DAEMON_STATUS_ID, " StartDaemon ", true).setValueChangedListener(
				new ValueChangedListener<Boolean>() {
					@Override
					public void onValueChanged(Boolean value) {
						if(value == true) {
							System.out.println("Starting Daemon Again!");
							applyDesiredDaemonStatus();
						}						
					}
		});;
	}

	// This method updates the parameters of the registered vacuum capability for all individual grippers
	private void updateVacuumCapability(boolean useFragileHandling) {
		if (useFragileHandling) {
			gripVacuumCapability.updateCapability(0, 70, 40, Pressure.Unit.KPA);
		} else {
			gripVacuumCapability.updateCapability(0, 100, 70, Pressure.Unit.KPA);
		}
	}

	@Override
	public void generatePreambleScript(ScriptWriter scriptWriter) {
		scriptWriter.appendLine("modbus_xmlrpc = rpc_factory(\"xmlrpc\",\"http://127.0.0.1:50408/RPC2\")");
		scriptWriter.appendLine("isConnected = modbus_xmlrpc.reachable()");		
		scriptWriter.appendLine("if ( isConnected != True):");
		scriptWriter.appendLine("    popup(\"Modbus xmlrpc is not available!\")");
		scriptWriter.appendLine("end");
				
				 
				
	}

	@Override
	public void generateGripActionScript(ScriptWriter scriptWriter, GripActionParameters gripActionParameters) {
		System.out.println("Grip Action :");

		printFragileHandlingSelection();
		if (fragileHandlingInput.getValue()){
			// Simulate applying fragile handling
			scriptWriter.appendLine("sleep(0.01)");
		}

		SelectableGripper selectedGripper = gripActionParameters.getGripperSelection();
		printSelectedGripper(selectedGripper);

		if (zoneAGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("set_tool_digital_out(0, True)");
		} else if (zoneBGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("set_tool_digital_out(1, True)");
		} else if (zoneABGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("set_tool_digital_out(0, True)");
			scriptWriter.appendLine("set_tool_digital_out(1, True)");
		}
	}

	@Override
	public void generateReleaseActionScript(ScriptWriter scriptWriter, ReleaseActionParameters releaseActionParameters) {
		System.out.println("Release Action :");

		printFragileHandlingSelection();
		if (fragileHandlingInput.getValue()){
			// Simulate applying fragile handling
			scriptWriter.appendLine("sleep(0.01)");
		}

		SelectableGripper selectedGripper = releaseActionParameters.getGripperSelection();
		printSelectedGripper(selectedGripper);

		if (zoneAGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("set_tool_digital_out(0, False)");
		} else if (zoneBGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("set_tool_digital_out(1, False)");
		} else if (zoneABGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("set_tool_digital_out(0, False)");
			scriptWriter.appendLine("set_tool_digital_out(1, False)");
		}
	}

	private void printSelectedGripper(SelectableGripper selectedGripper) {
		System.out.println("Selected Gripper: " + selectedGripper.getDisplayName() + "\n");
	}

	private void printFragileHandlingSelection() {
		System.out.println("Using Fragile Handling: " + fragileHandlingInput.getValue());
	}
	
	
	
	public ModbusDaemonInterface getXmlRpcDaemonInterface() {
		return this.modbusDaemonInterface;
	}
	
}