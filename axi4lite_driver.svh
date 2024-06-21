////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

class axi4lite_driver extends uvm_driver #(axi4lite_transaction);
  `uvm_component_utils(axi4lite_driver)

  function new (string name, uvm_component parent);
    super.new(name, parent);
  endfunction // new

  // The cfg configuration below is not used but is made available for future use
  axi4lite_config cfg;
  // TODO: Find a better way of parameterizing or get rid of the parameter
  virtual axi4lite_interface#(`ADDR_WIDTH) vif;

  function void build_phase (uvm_phase phase);
    super.build_phase(phase);


    // Getting the virtual interface vif
    if (!uvm_config_db#(virtual axi4lite_interface)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", {"virtual interface must be set for: ", get_full_name(), ".vif"});
  endfunction // build_phase

  virtual task run_phase (uvm_phase phase);
    fork
      get_and_drive();
      reset_signals();
    join
  endtask // run_phase

  virtual protected task get_and_drive();
    forever begin
      @(posedge vif.aclk);
      if (vif.areset_n == 1'b0) begin
        @(posedge vif.areset_n);
        @(posedge vif.aclk);
      end
      seq_item_port.get_next_item(req);
      `uvm_info("DRV", req.convert2string(), UVM_LOW)
      repeat(req.cycles) begin
        @(posedge vif.aclk);
      end
      drive_transfer(req);
      seq_item_port.item_done();
    end
  endtask // get_and_drive

// TODO: try one of the options below to replace 0 in awaddr and araddr
// SystemVerilog will resize numeric literals to the correct size following well-defined rules so its not necessary to define the size:

// logic [1:0][BITWIDTH-1:0] x = '{'h30, 'h40};
// However, some tools do throw warnings so you can cast the literal to the right size like so:

// logic [1:0][BITWIDTH-1:0] x = '{BITWIDTH'('h30), BITWIDTH'('h40)}
// or try `h0 or `b0

  virtual protected task reset_signals();
    forever begin
      // TODO: Possibly change conditions to posedge clk and areset_n = 0
      @(negedge vif.areset_n);
        // vif.GPIN    <= 32'h0;
        vif.AWADDR  <=     0;
        vif.AWPROT  <=  3'h0;
        vif.AWVALID <=  1'b0;
        vif.WDATA   <= 32'h0;
        vif.WSTRB   <=  4'h0;
        vif.WVALID  <=  1'b0;
        vif.BREADY  <=  1'b1;
        vif.ARADDR  <=     0;
        vif.ARPROT  <=  3'h0;
        vif.ARVALID <=  1'b0;
        vif.RREADY  <=  1'b1;
    end
  endtask // reset_signals

  // drive_transfer
  virtual protected task drive_transfer (axi4lite_transaction txn);
    fork
      drive_address_phase(txn);
      drive_data_phase(txn);
    join
  endtask : drive_transfer

  // drive_address_phase
  virtual protected task drive_address_phase (axi4lite_transaction txn);
    `uvm_info("axi4lite_master_driver", "drive_address_phase",UVM_HIGH);
    case (txn.trans)
      READ : drive_read_address_channel(txn);
      WRITE: drive_write_address_channel(txn);
    endcase
  endtask : drive_address_phase

  // drive_data_phase
  virtual protected task drive_data_phase (axi4lite_transaction txn);
    bit[31:0] rw_data;
    bit err;

    rw_data = txn.data;
    case (txn.trans)
      READ : drive_read_data_channel(rw_data, err);
      WRITE: drive_write_data_channel(rw_data, err);
    endcase     
  endtask : drive_data_phase

  virtual protected task drive_write_address_channel (axi4lite_transaction txn);
    int to_ctr;

    // Put the address on the address line and signal it is valid
    vif.AWADDR  <= txn.addr;
    vif.AWPROT  <= 3'h0;
    vif.AWVALID <= 1'b1;
    // Start a loop while waiting for DUT to signal it is ready for the write address
    for(to_ctr = 0; to_ctr <= 31; to_ctr ++) begin
      @(negedge vif.aclk);
      if (vif.AWREADY) break;
    end
    // If the loop drops out because the counter gets to 31 then flag a timeout error
    // The DUT did not signal it is ready for the write address
    if (to_ctr == 31) begin
      `uvm_error("axi4lite_master_driver","AWVALID timeout");
    end
    // Drive the address and valid low at the next clock edge. The DUT should have registered the write address.
    @(posedge vif.aclk);
    vif.AWADDR  <= 0;
    vif.AWPROT  <= 3'h0;
    vif.AWVALID <= 1'b0;     
  endtask : drive_write_address_channel

  virtual protected task drive_read_address_channel (axi4lite_transaction txn);
    int to_ctr;

    // Put the address on the address line and signal it is valid
    vif.ARADDR  <= txn.addr;
    vif.ARPROT  <= 3'h0;
    vif.ARVALID <= 1'b1;
    // Start a loop while waiting for DUT to signal it is ready for the read address
    for(to_ctr = 0; to_ctr <= 31; to_ctr++) begin
      @(negedge vif.aclk);
      if (vif.ARREADY) break;
    end
    // If the loop drops out because the counter gets to 31 then flag a timeout error
    // The DUT did not signal it is ready for the read address
    if (to_ctr == 31) begin
      `uvm_error("axi4lite_master_driver","ARVALID timeout");
    end       
    // Drive the address and valid low at the next clock edge. The DUT should have registered the read address.
    @(posedge vif.aclk);
    vif.ARADDR  <= 0;
    vif.ARPROT  <= 3'h0;
    vif.ARVALID <= 1'b0;      
  endtask : drive_read_address_channel

  // drive write data channel
  virtual protected task drive_write_data_channel (bit[31:0] data, output bit error);
    int to_ctr;
    
    // Put the data on the data line and signal it is valid
    vif.WDATA  <= data;
    vif.WSTRB  <= 4'hf;
    vif.WVALID <= 1'b1;
    @(posedge vif.aclk);
    // Start a loop while waiting for DUT to signal it is ready for the write data
    for(to_ctr = 0; to_ctr <= 31; to_ctr++) begin
      @(negedge vif.aclk);
      if (vif.WREADY) break;
    end
    // If the loop drops out because the counter gets to 31 then flag a timeout error
    // The DUT did not signal it is ready for the write data
    if (to_ctr == 31) begin
      `uvm_error("axi4lite_master_driver","WVALID timeout");
    end
    // Drive the data and valid low at the next clock edge. The DUT should have registered the write data.
    @(posedge vif.aclk);
    vif.WDATA  <= 32'h0;
    vif.WSTRB  <= 4'h0;
    vif.WVALID <= 1'b0;

    //wait for write response
    for(to_ctr = 0; to_ctr <= 31; to_ctr ++) begin
      @(posedge vif.aclk);
      if (vif.BVALID) break;
    end
    if (to_ctr == 31) begin
      `uvm_error("axi4lite_master_driver","BVALID timeout");
    end
    else begin
      if (vif.BVALID == 1'b1 && vif.BRESP != 2'h0)
        `uvm_error("axi4lite_master_driver","Received ERROR Write Response");
      // TODO: Check waveform on bready
      vif.BREADY <= vif.BVALID;
      @(posedge vif.aclk);
    end
  endtask : drive_write_data_channel

  // drive read data channel
  virtual protected task drive_read_data_channel (output bit [31:0] data, output bit error);
    int to_ctr;

    for(to_ctr = 0; to_ctr <= 31; to_ctr ++) begin
        @(negedge vif.aclk);
        if (vif.RVALID) break;
    end
    
    data = vif.RDATA;
    
    if (to_ctr == 31) begin
      `uvm_error("axi4lite_master_driver","RVALID timeout");
    end
    else begin
      if (vif.RVALID == 1'b1 && vif.RRESP != 2'h0) `uvm_error("axi4lite_master_driver","Received ERROR Read Response");

      vif.RREADY <= vif.RVALID;
      @(posedge vif.aclk);
    end
  endtask : drive_read_data_channel

endclass // axi4lite_driver
