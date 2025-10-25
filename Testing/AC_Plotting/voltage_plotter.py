import csv
import matplotlib.pyplot as plt

def plot_voltage_from_csv(csv_filename):
    # Read the CSV file
    with open(csv_filename, 'r') as file:
        csv_reader = csv.reader(file)
        voltage_values = next(csv_reader)  # Read the first line
        
        # Read the second line for time scale
        try:
            time_values = next(csv_reader)  # Read the second line
            times = [float(t) for t in time_values]
            use_time_scale = True
        except StopIteration:
            # No second line, use point indices
            times = list(range(len(voltage_values)))
            use_time_scale = False
    
    # Convert string values to floats
    voltages = [float(v) for v in voltage_values]
    
    # Convert from digital to voltage
    for i in range(0, len(voltages)):
        if csv_filename == "adc.csv":
            voltages[i] = (voltages[i]/8191)*3.3  # for adc
        else:
            voltages[i] = (voltages[i]/4095)*5  # for dac
    
    # Create the plot
    plt.figure(figsize=(10, 6))
    plt.plot(times, voltages, marker='o', linestyle='-', linewidth=2, markersize=4)
    
    if use_time_scale:
        plt.xlabel('Time')
    else:
        plt.xlabel('Point Index')
    
    plt.ylabel('Voltage (V)')
    plt.title('Voltage vs Time' if use_time_scale else 'Voltage vs Point Index')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Save plot
    #plt.savefig('voltage_plot.png', dpi=300, bbox_inches='tight')
    #print(f"Plot saved as 'voltage_plot.png'")
    
    # Display plot
    plt.show()
    
    # Print some statistics
    print(f"\nTotal points: {len(voltages)}")
    print(f"Min voltage: {min(voltages):.2f} V")
    print(f"Max voltage: {max(voltages):.2f} V")
    print(f"Average voltage: {sum(voltages)/len(voltages):.2f} V")

if __name__ == "__main__":
    csv_file = input("File name: ")
    plot_voltage_from_csv(csv_file)