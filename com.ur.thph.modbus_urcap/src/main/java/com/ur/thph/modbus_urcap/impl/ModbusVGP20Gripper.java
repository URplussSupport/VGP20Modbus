package com.ur.thph.modbus_urcap.impl;

import com.ur.urcap.api.contribution.DaemonContribution;
import com.ur.urcap.api.contribution.driver.general.tcp.TCPConfiguration;
import com.ur.urcap.api.contribution.driver.general.userinput.CustomUserInputConfiguration;
import com.ur.urcap.api.contribution.driver.general.userinput.TextComponent;
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

import javax.swing.Icon;
import javax.swing.ImageIcon;
import java.util.Locale;
import java.util.Timer;


public class ModbusVGP20Gripper implements GripperContribution {

	//private static final String GRIPPER_TITLE = "Dual Zone Gripper";

;

	// Ids must remain constant over time and versions of the Gripper URCap, since they are used for persistence and
	// can be used by other URCaps for configuring Gripper program nodes.
	

	private static final String FRAGILE_HANDLING_LABEL = "Use Fragile Handling";
	private static final String FRAGILE_HANDLING_ID = "fragile_handling_id";

	
	private GripVacuumCapability gripVacuumCapability;

	private BooleanUserInput fragileHandlingInput;
	
	/*
	 * ================================================================================================================
	 * VGP20 - Hackathon Stuff
	 * ================================================================================================================
	 */
	
	private Timer uiTimer;
	private boolean pauseTimer;
	private final ModbusDaemonService modbusDaemonService;
	private ModbusDaemonInterface modbusDaemonInterface;
	
	private static final int PORT = 50408;						// Port Changed from 40408 -> to 50408
	private static final String HOST = "127.0.0.1";
	
	private static final String SCRIPT_FILE_PATH_MODBUS = "/script/modbus.script";
	private static final String SCRIPT_FILE_PATH_VGP20 = "/script/vgp20_scripts.script";
	
	private static final String CHANNEL_1_ID = "Channel1_Id";
	private static final String CHANNEL_2_ID = "Channel2_Id";
	private static final String CHANNEL_3_ID = "Channel3_Id";
	private static final String CHANNEL_4_ID = "Channel4_Id";
	private static final String CHANNEL_ALL_ID = "ChannelAll_Id";
	
	private static final String CHANNEL_1_NAME = "Channel 1";
	private static final String CHANNEL_2_NAME = "Channel 2";
	private static final String CHANNEL_3_NAME = "Channel 3";
	private static final String CHANNEL_4_NAME = "Channel 4";
	private static final String CHANNEL_ALL_NAME = "All Channels";
	
	private static final String CHANNEL_TCP = "VPG20_TCP";	
	
	private static final String GRIPPER_TITLE = "V";
	
	private SelectableGripper channel1Gripper;
	private SelectableGripper channel2Gripper;
	private SelectableGripper channel3Gripper;
	private SelectableGripper channel4Gripper;
	private SelectableGripper channelAllGripper;
	
	private String maximumCurrentToolIO = "0.6";  
	
	private ScriptHandler handler;
	
	private final String DAEMON_STATUS_LABEL = "daemonStatusLabel";
	private final String DAEMON_STATUS_ID = "startDaemonCheckBox";
	
	
	private ControllableResourceModel resourceModel;
	private ToolIOController toolIOController ;
	
	
	
	/*
	 * ================================================================================================================
	 */
	

	public ModbusVGP20Gripper(ModbusDaemonService modbusDaemonService) {
		this.modbusDaemonService = modbusDaemonService;
		this.modbusDaemonInterface = new ModbusDaemonInterface(HOST, PORT);		
		this.handler = new ScriptHandler(); 		
		applyDesiredDaemonStatus();
	}

	
	
	private void applyDesiredDaemonStatus() {
		new Thread(new Runnable() {
			@Override
			public void run() {
				if (isDaemonEnabled().booleanValue()) {
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
				
				channel1Gripper = gripperListBuilder.createGripper(CHANNEL_1_ID, CHANNEL_1_NAME, true);
				channel2Gripper = gripperListBuilder.createGripper(CHANNEL_2_ID, CHANNEL_2_NAME, true);
				channel3Gripper = gripperListBuilder.createGripper(CHANNEL_3_ID, CHANNEL_3_NAME, true);
				channel4Gripper = gripperListBuilder.createGripper(CHANNEL_4_ID, CHANNEL_4_NAME, true);
				channelAllGripper = gripperListBuilder.createGripper(CHANNEL_ALL_ID, CHANNEL_ALL_NAME, true);
			
				
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
		setMaxCurrentToolIO( getRobotType( gripperAPIProvider ) );
	}

	
	public boolean getResourceControlStatus() {
		return this.toolIOController.hasControl();
	}
	
	private void setMaxCurrentToolIO(RobotType type) {
		if(type == RobotType.UR3) {
			this.maximumCurrentToolIO = "0.6";			
		} else if (type == RobotType.UR5) {
			this.maximumCurrentToolIO = "1.5";
		} else if (type == RobotType.UR10 || type == RobotType.UR16 ) {
			this.maximumCurrentToolIO = "2.0";
		} else {
			this.maximumCurrentToolIO = "0.6";
		}
	}
	
	public String getMaxCurrentToolIO() {
		return this.maximumCurrentToolIO;
	}
	
	private String toolControlStatusText() {
		String statusText;
		if( getResourceControlStatus() ) {
			statusText = " The VGP20 has control over the Tool Interface!";
			return statusText;
		}else {
			statusText = "The VGP20 does not have control over the Tool Interface!";
			return statusText;
		}
		
	}
	
	private ImageIcon getToolStatusIcon() {		
		if(getResourceControlStatus()) {
			return new ImageIcon(getClass().getResource("/logo/approve.png"));
		} else {
			return new ImageIcon(getClass().getResource("/logo/warning_icon_small.png"));
		}
		
		
	}
	
	public RobotType getRobotType(GripperAPIProvider gripperAPIProvider) {
		return gripperAPIProvider.getSystemAPI().getRobotModel().getRobotType();
	}
	
	
	
	private void configureGripperTCPs(SystemConfiguration systemConfiguration, GripperAPIProvider gripperAPIProvider) {		
		PoseFactory poseFactory = gripperAPIProvider.getPoseFactory();
		
		TCPConfiguration zoneATCPConfiguration = systemConfiguration.getTCPConfiguration(channel1Gripper);
		zoneATCPConfiguration.setTCP(CHANNEL_TCP, poseFactory.createPose(0, 0, 50, 0, 0, 0, Length.Unit.MM, Angle.Unit.DEG));
	}

	private void customizeInstallationScreen(CustomUserInputConfiguration configurationUIBuilder) {
		fragileHandlingInput = configurationUIBuilder.registerBooleanInput(FRAGILE_HANDLING_ID, FRAGILE_HANDLING_LABEL, false);
		fragileHandlingInput.setValueChangedListener(new ValueChangedListener<Boolean>() {
			@Override
			public void onValueChanged(Boolean useFragileHandling) {
				updateVacuumCapability(useFragileHandling);
			}
		});
		
		
		/*
		 * TODO Adjust label integration to display the correct value directly after starting the Daemon in the beginning
		 */		
		final TextComponent daemonStatusUI = configurationUIBuilder.addText("The Daemon is ",   getDaemonState() +" Perfect!"); 
		configurationUIBuilder.registerBooleanInput(DAEMON_STATUS_ID, " StartDaemon ", true).setValueChangedListener(
				new ValueChangedListener<Boolean>() {
					@Override
					public void onValueChanged(Boolean value) {
						if(value == true) {
							System.out.println("Starting Daemon Again!");
							applyDesiredDaemonStatus();
							daemonStatusUI.setText(getDaemonState() +" Perfect!");
						}						
					}
		});;
		
		/*
		 * TODO Probably due to a timing issue showcasing the status of the IO Control leads to a Nullpointer exception
		 * configurationUIBuilder.addText("Tool Control Status:", getToolStatusIcon(), toolControlStatusText());// -> Throwing Null Pointer!
		 */
		
	}

	// This method updates the parameters of the registered vacuum capability for all individual grippers
	private void updateVacuumCapability(boolean useFragileHandling) {
		if (useFragileHandling) {
			gripVacuumCapability.updateCapability(0, 40, 30, Pressure.Unit.KPA);
		} else {
			gripVacuumCapability.updateCapability(0, 60, 50, Pressure.Unit.KPA);
		}
	}

	@Override
	public void generatePreambleScript(ScriptWriter scriptWriter) {
		scriptWriter.appendLine("vgp20_max_tool_current = " + getMaxCurrentToolIO()); // The maximum current available is dependent on the used robot size
		
		String scriptCode_modbus = this.handler.readScriptFile(SCRIPT_FILE_PATH_MODBUS, scriptWriter);
		String scriptCode_VGP20 = this.handler.readScriptFile(SCRIPT_FILE_PATH_VGP20, scriptWriter);				
	}

	@Override
	public void generateGripActionScript(ScriptWriter scriptWriter, GripActionParameters gripActionParameters) {
		System.out.println("Grip Action :");
		
		/*
		 * 
		 * 
		 * 
		printFragileHandlingSelection();
		if (fragileHandlingInput.getValue()){
			// Simulate applying fragile handling
			scriptWriter.appendLine("sleep(0.01)");
		}
		*/
		
		SelectableGripper selectedGripper = gripActionParameters.getGripperSelection();
		printSelectedGripper(selectedGripper);
		
		Pressure pressure = gripActionParameters.getVacuum();
		
		scriptWriter.appendLine("vgp20TmpPressure = " + pressure.getAs(Pressure.Unit.KPA));
		
		if(channel1Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [True, False, False, False]");
		} else if (channel2Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [False, True, False, False]");
		} else if (channel3Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [False, False, True, False]");
		} else if (channel4Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [False, False, False, True]");
		} else if (channelAllGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [True, True, True, True]");
		}
				
			
		scriptWriter.appendLine("vgp20_grip(vgp20TmpChList, vgp20TmpPressure  )");
		

	}

	@Override
	public void generateReleaseActionScript(ScriptWriter scriptWriter, ReleaseActionParameters releaseActionParameters) {
		System.out.println("Release Action :");
		/*
		 * 
		 * 
		 * 
		printFragileHandlingSelection();
		if (fragileHandlingInput.getValue()){
			// Simulate applying fragile handling
			scriptWriter.appendLine("sleep(0.01)");
		}
		*/
		SelectableGripper selectedGripper = releaseActionParameters.getGripperSelection();
		printSelectedGripper(selectedGripper);

		if(channel1Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [True, False, False, False]");
		} else if (channel2Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [False, True, False, False]");
		} else if (channel3Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [False, False, True, False]");
		} else if (channel4Gripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [False, False, False, True]");
		} else if (channelAllGripper.equals(selectedGripper)) {
			scriptWriter.appendLine("vgp20TmpChList = [True, True, True, True]");
		}
		
		scriptWriter.appendLine("vgp20_release(vgp20TmpChList)");
		
	}

	private void printSelectedGripper(SelectableGripper selectedGripper) {
		System.out.println("Selected Gripper: " + selectedGripper.getDisplayName() + "\n");
	}

	private void printFragileHandlingSelection() {
		/*
		 * TODO Is Fragile Handling needed for our cause?
		 */
		
		
		System.out.println("Using Fragile Handling: " + fragileHandlingInput.getValue());
	}
	
	
	
	public ModbusDaemonInterface getXmlRpcDaemonInterface() {
		return this.modbusDaemonInterface;
	}
	
}