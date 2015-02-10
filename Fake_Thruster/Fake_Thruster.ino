void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}

//**** Function Declarations ****
unsigned short readRegister(byte);
int setRegister(byte, unsigned short);
void printHex(byte hex);  //prints hex values with leading zeros

//**** Register Names ****
unsigned short setup_;
unsigned short hvdac_;
unsigned short thrust_gap_time_;
unsigned short thrust_timeout_;
unsigned short thrust_reverse_time_;
unsigned short thrust_register_;
unsigned short hv_setpoint_;
unsigned short test_register_;
unsigned short status_ = 0x0800;
unsigned short fpga_rev_reg_low_;
unsigned short fpga_rev_reg_high_;
unsigned short adc0_;
unsigned short adc1_;
unsigned short adc2_;
unsigned short adc3_;
unsigned short adc4_;
unsigned short adc5_;
unsigned short adc6_;
unsigned short adc7_;
unsigned short controller_gain_;
unsigned short hvdac_limit_;
unsigned short hv_setpoint_limit_;
unsigned short btv_addr_set_;
unsigned short btv_wrt_port_;
unsigned short cmd_wrt_port_;
unsigned short hv_diff_set_;
unsigned short auto_period_set_;
unsigned short auto_pulse_width_set_;
unsigned short auto_period_read_;
unsigned short auto_pulse_width_read_;
unsigned short serial_port_force_;

int i;
byte read_flag;
byte set_reg;
byte read_reg;
unsigned short set_val;
unsigned short data;
byte in_byte;
byte command_array[3];
int command;
int count;
int set_flag;
void loop() {
  // put your main code here, to run repeatedly: 
  if (Serial1.available())
  {
    command_array[0] = Serial1.read();
    read_flag = command_array[0] & 0x01;  //check bit 0
    
    if (command_array[0] == 0xFF && command_array[1] == 0xFF && command_array[2] == 0xFF)
    {//Reset Buffer
      Serial.println("Clearing Serial1 Buffer...");
      while (Serial1.available())
        Serial1.read();
    }//end else if
    
    else if ((read_flag == 0x00) || (command_array[0] == 0x30))
    {//Write Command
      Serial.print("(W): ");
      count = 0;
      while (Serial1.available() < 2)
      {
        count++;
        if (count % 10 == 0) 
          Serial.print(".");
      }//end while
      
      command_array[1] = Serial1.read();
      command_array[2] = Serial1.read();    
      
      //Echo back to Thruster Board
      Serial1.write(command_array[0]);
      Serial1.write(command_array[2]);  //send MSB first 
      Serial1.write(command_array[1]);  //send LSB last
      
      set_reg = command_array[0] >> 1;
      set_val = 0;  //ensures not left over bits
      set_val = command_array[1];  //MSB
      set_val = (set_val << 8) | command_array[2];  //LSB
      set_flag = setRegister(set_reg, set_val);
      
      if (set_flag == 1)
      {
        Serial.print("Success! Reg: 0x");
        printHex(command_array[0] >> 1);
        Serial.print(" = 0x"); 
        printHex(command_array[1]);
        printHex(command_array[2]); 
        Serial.println();
      }//end if
    }//end if
    
    else if (read_flag == 0x01)
    {//Read Command
      Serial.print("(R): ");
      command_array[1] = 0x00; //reset to 0x00
      command_array[2] = 0x00; //reset to 0x00 
      
      read_reg = command_array[0] >> 1;
      
      Serial.print("Reg: 0x");
      printHex(read_reg);
      data = readRegister(read_reg);
      command_array[2] |= data;  //LSB
      command_array[1] |= data >> 8;  //MSB
      
      Serial.print(" | Data: 0x");
      printHex(command_array[0]);
      Serial.print(" ");
      printHex(command_array[1]);
      Serial.print(" ");
      printHex(command_array[2]); 
      Serial.println();
      
      //Echo back to Thruster Board
      Serial1.write(command_array[0]);
      Serial1.write(command_array[1]);
      Serial1.write(command_array[2]);
    }//end else if
    
    
    else
    {
      while (Serial1.available())
        Serial1.read();
      command_array[1] = 0x00; //Fake response byte
      command_array[2] = 0x00; //Fake response byte
      Serial.println("Something went bad!");
    }//end else
    
  }//end if
    
}//end main()



unsigned short readRegister(byte reg)
{
  switch (reg) {
    case 0x00:  //SETUP
      return setup_;
    case 0x02:  //HVDAC
      return hvdac_;
    case 0x03:  //THRUST_GAP_TIME
      return thrust_gap_time_;
    case 0x04:  //THRUST_TIMEOUT
      return thrust_timeout_;
    case 0x05:  //THRUST_REVERSE_TIME
      return thrust_reverse_time_;
    case 0x07:  //THRUST_REGISTER
      return thrust_register_;
    case 0x08:  //HV_SETPOINT
      return hv_setpoint_;
    case 0x0C:  //TEST_REGISTER
      return test_register_;
    case 0x0D:  //STATUS
      return status_;
    case 0x0E:  //FPGA_REV_REG_LOW
      return fpga_rev_reg_low_;
    case 0x0F:  //FPGA_REV_REG_HIGH
      return fpga_rev_reg_high_;
    case 0x10:  //ADC0
      return adc0_;
    case 0x11:  //ADC1
      return adc1_;
    case 0x12:  //ADC2
      return adc2_;
    case 0x13:  //ADC3
      return adc3_;
    case 0x14:  //ADC4
      return adc4_;
    case 0x15:  //ADC5
      return adc5_;
    case 0x16:  //ADC6
      return adc6_;
    case 0x17:  //ADC7
      return adc7_;
    case 0x1A:  //CONTROLLER_GAIN
      return controller_gain_;
    case 0x1B:  //HVDAC_LIMIT
      return hvdac_limit_;
    case 0x1C:  //HV_SETPOINT_LIMIT
      return hv_setpoint_limit_;
    case 0x23:  //HV_DIFF_SET
      return hv_diff_set_;
    case 0x26:  //AUTO_PERIOD_SET
      return auto_period_set_;
    case 0x27:  //AUTO_PULSE_WIDTH_SET
      return auto_pulse_width_set_;
    case 0x28:  //AUTO_PERIOD_READ
      return auto_period_read_;
    case 0x29:  //AUTO_PULSE_WIDTH_READ
      return auto_pulse_width_read_;
    default:
      Serial.print("Cannot read register: ");
      Serial.print("0x");
      printHex(reg);
      Serial.println();
      return 0;
  }//end switch
}//end readRegister()
      
    
int setRegister(byte reg, unsigned short value)
{
  switch (reg) {
    case 0x00:  //SETUP
      setup_ = value;
      return 1;
    case 0x02:  //HVDAC
      if (value > 0 && value <= hvdac_limit_) {
        hvdac_ = value;
        return 1;
      }//end if
      return 0;
    case 0x03:  //THRUST_GAP_TIME
      thrust_gap_time_ = value;
      return 1;
    case 0x04:  //THRUST_TIMEOUT
      thrust_timeout_ = value;
      return 1;
    case 0x05:  //THRUST_REVERSE_TIME
      thrust_reverse_time_ = value;
      return 1;
    case 0x07:  //THRUST_REGISTER
      thrust_register_ = value;
      return 1;
    case 0x08:  //HV_SETPOINT
      if (value >= 0 && value <= hv_setpoint_limit_) {
        hv_setpoint_ = value;
        return 1;
      }//end if
      return 0;
    case 0x0C:  //TEST_REGISTER
      test_register_ = value;
      return 1;
    case 0x0D:  //STATUS
      status_ ^= value;
      return 1;
    case 0x1A:  //CONTROLLER_GAIN
      controller_gain_ = value;
      return 1;
    case 0x1B:  //HVDAC_LIMIT
      hvdac_limit_ = value;
      return 1;
    case 0x1C:  //HV_SETPOINT_LIMIT
      hv_setpoint_limit_ = value*65536/455.05/4.096;
      return 1;
    case 0x20:  //BTV_ADDR_SET
      btv_addr_set_ = value;
      return 1;
    case 0x21:  //BTV_WRT_PORT
      btv_wrt_port_ = value;
      return 1;
    case 0x22:  //CMD_WRT_PORT
      cmd_wrt_port_ = value;
      return 1;
    case 0x23:  //HV_DIFF_SET
      hv_diff_set_ = value;
      return 1;
    case 0x26:  //AUTO_PERIOD_SET
      auto_period_set_ = value;
      return 1;
    case 0x27:  //AUTO_PULSE_WIDTH_SET
      auto_pulse_width_set_ = value;
      return 1;
    case 0x30:  //SERIAL_PORT_FORCE
      serial_port_force_ = value;
      return 1;
    default:
      Serial.print("Cannot set register: ");
      Serial.print("0x");
      printHex(reg);
      Serial.println();
      return 0;
  }//end switch
  
}//end setRegister()

void printHex(byte hex) 
{
  if (hex < 0x10) {
    Serial.print("0");
    Serial.print(hex, HEX);
  }//end if
  
  else
    Serial.print(hex, HEX);
}//end printHex()

