import sys
import csv
import time
import signal
import datetime
import argparse

import numpy as np
from matplotlib import pyplot as plt
import SoapySDR
from SoapySDR import *

def handler(signum, frame):
    global running 

    print("ending...")
    running = False

def main(rx_chan, fs, bw, freq, gain, poll_time):
    global running
    ############################################################################################
    # Settings
    ############################################################################################
    running = True
    # Data transfer settings
    N = 16384               # Number of complex samples per transfer
    use_agc = False          # Use or don't use the AGC
    timeout_us = int(10e6)
    rx_bits = 12
    lo_freq = freq + (bw/2)

    header = 'Time, Peak Frequency MHz, Peak Power dB \n'

    signal.signal(signal.SIGINT, handler)

    sdr = SoapySDR.Device()

    # Create data buffer and start streaming samples to it
    rx_buff = np.empty(2 * N, np.int16)                 # Create memory buffer for data stream

    ############################################################################################
    # Receive Signal
    ############################################################################################

    f = open('output.csv', 'w', encoding='UTF8') 

    now =    datetime.datetime.now().strftime("%d-%b-%Y (%H:%M:%S.%f)")
    out = now + ', ' 
    row = 'time,' +  now  + '\n'
    f.write(row)
    print(row)
    row = 'sample rate, ' + "{:.0f}".format(fs) + '\n'
    f.write(row)
    print(row)
    row = 'Bandwidth, ' + "{:.0f}".format(bw) + '\n'
    f.write(row)
    print(row)
    row = 'Gain, ' + "{:d}".format(gain) + '\n'
    f.write(row)
    print(row)
    row = 'Poll Time, ' + "{:d}".format(poll_time) + '\n'
    f.write(row)
    print(row)
    row = 'Start Frequency, ' + "{:.0f}".format(freq) + '\n'
    f.write(row)
    print(row)
    row = 'End Frequency, ' + "{:.0f}".format(freq+bw) + '\n\n'
    f.write(row)
    print(row)

    row = "\nSweep From " +  str(int(freq)) + " to " + str( int(freq + bw)) +  "\n"
    print(row)

    f.write(header)
    """
        SOAPY_SDR_FATAL    
        SOAPY_SDR_CRITICAL 
        SOAPY_SDR_ERROR    
        SOAPY_SDR_WARNING  
        SOAPY_SDR_NOTICE   
        SOAPY_SDR_INFO     
        SOAPY_SDR_DEBUG    
        SOAPY_SDR_TRACE    
        SOAPY_SDR_SSI      
    """
#    SoapySDR.setLogLevel(SOAPY_SDR_DEBUG)
    SoapySDR.setLogLevel(SOAPY_SDR_NOTICE)

    sdr.setSampleRate(SOAPY_SDR_RX, rx_chan, fs)          # Set sample rate
    sdr.setBandwidth(SOAPY_SDR_RX, rx_chan, bw)          # Set sample rate

    sdr.setGain(SOAPY_SDR_RX, rx_chan, gain)

    sdr.setFrequency(SOAPY_SDR_RX, rx_chan, lo_freq)         # Tune the LO

    rx_stream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [rx_chan])  # Setup data stream

    while running:
        sdr.activateStream(rx_stream)  # this turns the radio on
        # Read the samples from the data buffer
        sr = sdr.readStream(rx_stream, [rx_buff], N, timeoutUs=timeout_us)
        sdr.deactivateStream(rx_stream)
        rc = sr.ret # number of samples read or the error code
        assert rc == N, 'Error Reading Samples from Device (error code = %d)!' % rc


        ############################################################################################
        # Plot Signal
        ############################################################################################
        # Convert interleaved shorts (received signal) to numpy.complex64 normalized between [-1, 1]
        s0 = rx_buff.astype(float) / np.power(2.0, rx_bits-1)
        s = (s0[::2] + 1j*s0[1::2])

        # Take the fourier transform of the signal and perform FFT Shift
        S = np.fft.fftshift(np.fft.fft(s, N) / N)
        S1 = 20*np.log10(np.abs(S))

        # Get the maximum element from a Numpy array
        maxElement = np.amax(S1)

        result = np.where(S1 == np.amax(S1))

        f_peak = (lo_freq + (np.arange(0, fs, fs/N) - (fs/2) + (fs/N))) 
        peak_freq = f_peak[result[0]]

        now =    datetime.datetime.now().strftime("%d-%b-%Y (%H:%M:%S.%f)")
        
        row = now + ', '  + "{:.0f}".format(peak_freq[0]) + ', ' + "{:.0f}".format(maxElement) + ' \n'
        f.write(row)
        print(header)
        print(row)
        time.sleep(poll_time)

    sdr.deactivateStream(rx_stream)
    sdr.closeStream(rx_stream)
    f.close()


def parse_command_line_arguments():
    """ Create command line options """
    help_formatter = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(description='scan for peaks, build a csv file ',
                                     formatter_class=help_formatter)
    parser.add_argument('-c', type=int, required=False, dest='chan',
                        default=0, help='Channel Number [0 or 1]')
    parser.add_argument('-s', type=float, required=False, dest='fs',
                        default=10e6, help='Sample Rate')
    parser.add_argument('-b', type=float, required=False, dest='bw',
                        default=8e6, help='Bandwidth')
    parser.add_argument('-f', type=float, required=False, dest='freq',
                        default=100e6, help='Starting Frequency')
    parser.add_argument('-g', type=int, required=False, dest='gain',
                        default=0, help='Gain (0 for automatic)')
    parser.add_argument('-p', type=int, required=False, dest='poll',
                        default=2, help='Time (in seconds) between frequency polls')
    return parser.parse_args(sys.argv[1:])


if __name__ == '__main__':
#    SoapySDR.setLogLevel(SOAPY_SDR_DEBUG)
    pars = parse_command_line_arguments()
    main(pars.chan, pars.fs, pars.bw, pars.freq, pars.gain, pars.poll)

