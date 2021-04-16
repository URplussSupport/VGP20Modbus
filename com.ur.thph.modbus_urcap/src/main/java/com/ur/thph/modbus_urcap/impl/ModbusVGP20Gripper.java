package com.ur.thph.modbus_urcap.impl;

import com.ur.urcap.api.contribution.DaemonContribution;
import com.ur.urcap.api.contribution.DaemonContribution.State;
import com.ur.urcap.api.contribution.driver.general.script.ScriptCodeGenerator;
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
import com.ur.urcap.api.contribution.driver.gripper.capability.GripDetectedParameters;
import com.ur.urcap.api.contribution.driver.gripper.capability.GripVacuumCapability;
import com.ur.urcap.api.contribution.driver.gripper.capability.GripperCapabilities;
import com.ur.urcap.api.contribution.driver.gripper.capability.GripperFeedbackCapabilities;
import com.ur.urcap.api.contribution.driver.gripper.capability.ReleaseDetectedParameters;
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

import java.awt.EventQueue;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;


public class ModbusVGP20Gripper implements GripperContribution {

	//private static final String GRIPPER_TITLE = "Dual Zone Gripper";

;

	// Ids must remain constant over time and versions of the Gripper URCap, since they are used for persistence and
	// can be used by other URCaps for configuring Gripper program nodes.
	

	private static final String FRAGILE_HANDLING_LABEL = "Use Fragile Handling";
	private static final String FRAGILE_HANDLING_ID = "fragile_handling_id";

	
	private GripVacuumCapability gripVacuumCapability;


	
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
	
	private static final String CHANNEL_A_ID = "ChannelA_Id";
	private static final String CHANNEL_B_ID = "ChannelB_Id";
	private static final String CHANNEL_C_ID = "ChannelC_Id";
	private static final String CHANNEL_D_ID = "ChannelD_Id";
	private static final String CHANNEL_ALL_ID = "ChannelAll_Id";
	
	private static final String CHANNEL_A_NAME = "Channel A";
	private static final String CHANNEL_B_NAME = "Channel B";
	private static final String CHANNEL_C_NAME = "Channel C";
	private static final String CHANNEL_D_NAME = "Channel D";
	private static final String CHANNEL_ALL_NAME = "All Channels";
	
	private static final String CHANNEL_TCP = "VPG20_TCP";	
	
	private static final String GRIPPER_TITLE = "VGP20 Gripper";
	
	private SelectableGripper channelAGripper;
	private SelectableGripper channelBGripper;
	private SelectableGripper channelCGripper;
	private SelectableGripper channelDGripper;
	private SelectableGripper channelAllGripper;
	
	private String maximumCurrentToolIO = "0.6";  
	
	private ScriptHandler handler;
	
	private final String DAEMON_STATUS_LABEL = "daemonStatusLabel";
	private final String DAEMON_STATUS_ID = "startDaemonCheckBox";
	
	private TextComponent daemonStatusUI;
	private TextComponent toolIOControlStatus;
	
	
	private ControllableResourceModel resourceModel;
	private ToolIOController toolIOController ;
	
	private boolean daemonInStartingProcess = false;
	
	
	
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
				if (isDaemonEnabled().booleanValue() && getDaemonState() != State.RUNNING && !daemonInStartingProcess) {
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
						daemonInStartingProcess = false;
					} 
				} else if (!isDaemonEnabled().booleanValue()){
					System.out.println("  DAEMON WILL BE STOPPED !! ");
					modbusDaemonService.getDaemon().stop();
				}
			}
		}).start();
	}
	
	private void awaitDaemonRunning(long timeOutMilliSeconds) throws InterruptedException {
		this.modbusDaemonService.getDaemon().start();
		long endTime = System.nanoTime() + timeOutMilliSeconds * 1000L * 1000L; 
		this.daemonInStartingProcess = true;
		while (System.nanoTime() < endTime
				&& (this.modbusDaemonService.getDaemon().getState() != DaemonContribution.State.RUNNING
						|| !modbusDaemonInterface.isReachable())) {

			Thread.sleep(100);

		}
		this.daemonInStartingProcess = false;
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
				
				channelAGripper = gripperListBuilder.createGripper(CHANNEL_A_ID, CHANNEL_A_NAME, true);
				channelBGripper = gripperListBuilder.createGripper(CHANNEL_B_ID, CHANNEL_B_NAME, true);
				channelCGripper = gripperListBuilder.createGripper(CHANNEL_C_ID, CHANNEL_C_NAME, true);
				channelDGripper = gripperListBuilder.createGripper(CHANNEL_D_ID, CHANNEL_D_NAME, true);
				channelAllGripper = gripperListBuilder.createGripper(CHANNEL_ALL_ID, CHANNEL_ALL_NAME, true);
			
				
				return gripperListBuilder.buildList();
			}
		});

		gripVacuumCapability = capabilities.registerGrippingVacuumCapability(0, 60, 40, Pressure.Unit.KPA);
		GripperFeedbackCapabilities feedbackCapabilites = gripperConfiguration.getGripperFeedbackCapabilities();
		
		feedbackCapabilites.registerGripDetectedCapability(new ScriptCodeGenerator<GripDetectedParameters>() {			
			@Override
			public void generateScript(ScriptWriter scriptWriter, GripDetectedParameters parameters) {
				SelectableGripper selectedGripper = parameters.getGripperSelection();
				scriptWriter.appendLine("return vgp20_grip_detect(" + getChannelListForScript( selectedGripper ) + ")" );				
			}
		});
		
		feedbackCapabilites.registerReleaseDetectedCapability(new ScriptCodeGenerator<ReleaseDetectedParameters>() {
			@Override
			public void generateScript(ScriptWriter scriptWriter, ReleaseDetectedParameters parameters) {
				SelectableGripper selectedGripper = parameters.getGripperSelection();
				scriptWriter.appendLine("vgp20_release_detect(" + getChannelListForScript( selectedGripper ) + " )");				
			}
		});
		
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
		if( this.toolIOController == null) {
			return false;
		} else {
			return this.toolIOController.hasControl();
		}
		
		
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
		if( getResourceControlStatus() ) 
			return "Control granted!";
		else 
			return "Please grant ToolIO control!";		
	}
	
	private ImageIcon getToolStatusIcon() {		
		if(getResourceControlStatus()) 
			return new ImageIcon(getClass().getResource("/logo/approve.png"));
		 else 
			return new ImageIcon(getClass().getResource("/logo/warning_icon_small.png"));
				
	}
	
	public RobotType getRobotType(GripperAPIProvider gripperAPIProvider) {
		return gripperAPIProvider.getSystemAPI().getRobotModel().getRobotType();
	}
	
	
	
	private void configureGripperTCPs(SystemConfiguration systemConfiguration, GripperAPIProvider gripperAPIProvider) {		
		PoseFactory poseFactory = gripperAPIProvider.getPoseFactory();
		
		TCPConfiguration generalTCPConfig = systemConfiguration.getTCPConfiguration(channelAGripper);
		generalTCPConfig.setTCP(CHANNEL_TCP, poseFactory.createPose(0, 0, 50, 0, 0, 0, Length.Unit.MM, Angle.Unit.DEG));
	}

	private void customizeInstallationScreen(CustomUserInputConfiguration configurationUIBuilder) {
		
		
		
		daemonStatusUI = configurationUIBuilder.addText("The Daemon is ",   getDaemonState() +" Perfect!"); 
		

		toolIOControlStatus = configurationUIBuilder.addText("Tool Control Status:", getToolStatusIcon(), toolControlStatusText());
		uiTimer = new Timer(true);
		uiTimer.schedule(new TimerTask() {
			@Override
			public void run() {
				EventQueue.invokeLater(new Runnable() {
					@Override
					public void run() {
						if (!pauseTimer) {
							updateUI();
							applyDesiredDaemonStatus();
						}
					}
				});
			}
		}, 0, 1000);
		 
		
	}
	private void updateUI() {
		if (daemonStatusUI != null && toolIOControlStatus != null) {
			daemonStatusUI.setText(getDaemonState() +" Perfect!");
			toolIOControlStatus.setText(toolControlStatusText());
			toolIOControlStatus.setIcon(getToolStatusIcon());
		} 
	}

	// This method updates the parameters of the registered vacuum capability for all individual grippers
	private void updateVacuumCapability(boolean useFragileHandling) {
		if (useFragileHandling) {
			gripVacuumCapability.updateCapability(0, 40, 30, Pressure.Unit.KPA);
		} else {
			gripVacuumCapability.updateCapability(0, 60, 40, Pressure.Unit.KPA);
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
		
	
		
		SelectableGripper selectedGripper = gripActionParameters.getGripperSelection();
		printSelectedGripper(selectedGripper);
		
		Pressure pressure = gripActionParameters.getVacuum();
		
		scriptWriter.appendLine("vgp20TmpPressure = " + pressure.getAs(Pressure.Unit.KPA));
		
				
			
		scriptWriter.appendLine("vgp20_grip(" + getChannelListForScript( selectedGripper ) + ", vgp20TmpPressure  )");
		

	}

	@Override
	public void generateReleaseActionScript(ScriptWriter scriptWriter, ReleaseActionParameters releaseActionParameters) {
		System.out.println("Release Action :");
	
		SelectableGripper selectedGripper = releaseActionParameters.getGripperSelection();
		scriptWriter.appendLine("vgp20_release(" + getChannelListForScript( selectedGripper ) + " )");
		
	}
	
	private String getChannelListForScript( SelectableGripper selectedGripper ) {
		String list = "";
		if(channelAGripper.equals(selectedGripper)) {
			list = "[True, False, False, False]";
		} else if (channelBGripper.equals(selectedGripper)) {
			list = "[False, True, False, False]";
		} else if (channelCGripper.equals(selectedGripper)) {
			list = "[False, False, True, False]";
		} else if (channelDGripper.equals(selectedGripper)) {
			list = "[False, False, False, True]";
		} else if (channelAllGripper.equals(selectedGripper)) {
			list = "[True, True, True, True]";
		}
		return list;		
	}
	

	private void printSelectedGripper(SelectableGripper selectedGripper) {
		System.out.println("Selected Gripper: " + selectedGripper.getDisplayName() + "\n");
	}

	
	public ModbusDaemonInterface getXmlRpcDaemonInterface() {
		return this.modbusDaemonInterface;
	}
	
}