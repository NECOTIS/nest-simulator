# -*- coding: utf-8 -*-
#
#  hpc_benchmark.py
#
#  This file is part of NEST.
#
#  Copyright (C) 2004 The NEST Initiative
#
#  NEST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 2 of the License, or
#  (at your option) any later version.
#
#  NEST is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with NEST.  If not, see <http://www.gnu.org/licenses/>.


'''
   This script produces a balanced random network of scale*11250 neurons in
   which the excitatory-excitatory neurons exhibit STDP with
   multiplicative depression and power-law potentiation. A mutual
   equilibrium is obtained between the activity dynamics (low rate in
   asynchronous irregular regime) and the synaptic weight distribution
   (unimodal). The number of incoming connections per neuron is fixed
   and independent of network size (indegree=11250).

   This is the standard network investigated in:
   Morrison et al (2007). Spike-timing-dependent plasticity in balanced random
     networks. Neural Comput 19(6):1437-67
   Helias et al (2012). Supercomputers ready for use as discovery machines for
     neuroscience. Front. Neuroinform. 6:26
   Kunkel et al (2014). Spiking network simulation code for petascale
     computers. Front. Neuroinform. 8:78

   A note on scaling
   -----------------

   This benchmark was originally developed for very large-scale simulations on
   supercomputers with more than 1 million neurons in the network and
   11.250 incoming synapses per neuron. For such large networks, input to a
   single neuron will be little correlated and network activity will remain
   stable for long periods of time.

   The original network size corresponds to a scale parameter of 100 or more.
   In order to make it possible to test this benchmark script on desktop
   computers, the scale parameter is set to 1 below, while the number of
   11.250 incoming synapses per neuron is retained. In this limit, correlations
   in input to neurons are large and will lead to increasing synaptic weights.
   Over time, network dynamics will therefore become unstable and all neurons
   in the network will fire in synchrony, leading to extremely slow simulation
   speeds.

   Therefore, the presimulation time is reduced to 50 ms below and the
   simulation time to 250 ms, while we usually use 100 ms presimulation and
   1000 ms simulation time.

   For meaningful use of this benchmark, you should use a scale > 10 and check
   that the firing rate reported at the end of the benchmark is below 10 spikes
   per second.
'''

import numpy as np
import os
# from scipy.special import lambertw
import sys
import time

import nest
import nest.raster_plot

M_INFO = 10
M_ERROR = 30


'''
 Parameter section

 define all relevant parameters: changes should be made here
'''

params = {
    'nvp': 1,               # total number of virtual processes
    'scale': 1.,           # scaling factor of the network size,
                            # total network size = scale*11250 neurons
    'simtime': 250.,        # total simulation time in ms
    'presimtime': 50.,      # simulation time until reaching equilibrium
    'dt': 0.1,              # simulation step
    'record_spikes': True,  # switch to record spikes of excitatory neurons to file
    'path_name': './',      # path where all files will have to be written
    'log_file': 'log',      # naming scheme for the log files
}

# -------------------------------------------------------------------------------------


def convert_synapse_weight(tau_m, tau_syn, C_m):
    '''Computes conversion factor for synapse weight from mV to pA. This
    function is specific to the used neuron model Leaky
    integrate-and-fire neuron with alpha-shaped postsynaptic currents.
    '''
    a = tau_m / tau_syn
    b = 1.0 / tau_syn - 1.0 / tau_m
    # time of maximum
    t_max = 1.0 / b * (-lambertwm1(-np.exp(-1.0 / a) / a).real - 1.0 / a)
    # maximum of PSP for current of unit amplitude
    v_max = np.exp(1.0) / (tau_syn * C_m * b) * ((np.exp(-t_max / tau_m) -
                                                  np.exp(-t_max / tau_syn)) / b - t_max * np.exp(-t_max / tau_syn))
    return 1. / v_max


# Rise time of synaptic currents
# The synaptic rise time is chosen such that the rise time of the
# evoked post-synaptic potential is 1.700759 ms.
# For alpha-shaped postynaptic currents, the rise time of the post-synaptic
# potential follows from the synaptic rise time as

# def PSP_rise_time(tau_m, tau_syn):
#     a = tau_m / tau_syn
#     b = 1.0 / tau_syn - 1.0 / tau_m

# Inverting this equation numerically leads to tau_syn = 0.32582722403722841 ms,
# as specified in model_params below.

tau_syn = 0.32582722403722841

# -------------------------------------------------------------------------------------

brunel_params = {
    'NE': int(9000 * params['scale']),  # number of excitatory neurons
    'NI': int(2250 * params['scale']),  # number of inhibitory neurons

    'Nrec': 100,  # number of neurons to record spikes from

    'model_params': {  # Set variables for iaf_psc_alpha
        'E_L': 0.0,  # Resting membrane potential(mV)
        'C_m': 250.0,  # Capacity of the membrane(pF)
        'tau_m': 10.0,  # Membrane time constant(ms)
        't_ref': 0.5,  # Duration of refractory period(ms)
        'V_th': 20.0,  # Threshold(mV)
        'V_reset': 0.0,  # Reset Potential(mV)
        'tau_syn_ex': tau_syn,  # time const. postsynaptic excitatory currents(ms)
        'tau_syn_in': tau_syn,  # time const. postsynaptic inhibitory currents(ms)
        'tau_minus': 30.0,  # time constant for STDP(depression)
        # V can be randomly initialized see below
        'V_m': 5.7  # mean value of membrane potential
    },

    'randomize_Vm': True,
    'mean_potential': 5.7,   # Note that Kunkel et al. (2014) report different values. The values
    'sigma_potential': 7.2,  # in the paper were used for the benchmarks on K, the values given
                             # here were used for the benchmark on JUQUEEN.

    'delay': 1.5,  # synaptic delay, all connections(ms)

    # synaptic weight
    'JE': 0.14,  # peak of EPSP

    'sigma_w': 3.47,  # standard dev. of E->E synapses(pA)
    'g': -5.0,

    'stdp_params': {
        'delay': 1.5,
        'alpha': 0.0513,
        'lambda': 0.1,  # STDP step size
        'mu': 0.4,  # STDP weight dependence exponent(potentiation)
        'tau_plus': 15.0,  # time constant for potentiation
    },

    'eta': 1.685,  # scaling of external stimulus
    'filestem': params['path_name']
}

'''
 FUNCTION SECTION
'''


def build_network(logger):
    tic = time.time()  # start timer on construction

    # unpack a few variables for convenience
    NE = brunel_params['NE']
    NI = brunel_params['NI']
    model_params = brunel_params['model_params']
    stdp_params = brunel_params['stdp_params']

    # set global kernel parameters
    nest.SetKernelStatus({
        'total_num_virtual_procs': params['nvp'],
        'resolution': params['dt'],
        'overwrite_files': True})

    nest.SetDefaults('iaf_psc_alpha', brunel_params['model_params'])

    nest.message(M_INFO, 'build_network', 'Creating excitatory population.')
    E_neurons = nest.Create('iaf_psc_alpha', NE)

    nest.message(M_INFO, 'build_network', 'Creating inhibitory population.')
    I_neurons = nest.Create('iaf_psc_alpha', NI)  # subnet gets own gid

    if brunel_params['randomize_Vm']:
        nest.message(M_INFO, 'build_network', 'Randomzing membrane potentials.')

        seed = nest.GetKernelStatus('rng_seeds')[-1] + 1 + nest.GetStatus([0], 'vp')[0]
        rng = np.random.RandomState(seed=seed)

        for node in get_local_nodes(E_neurons):
            nest.SetStatus([node],
                           {'V_m': rng.normal(brunel_params['mean_potential'], brunel_params['sigma_potential'])})

        for node in get_local_nodes(I_neurons):
            nest.SetStatus([node],
                           {'V_m': rng.normal(brunel_params['mean_potential'], brunel_params['sigma_potential'])})

    CE = int(1. * NE / params['scale'])  # number of incoming excitatory connections
    CI = int(1. * NI / params['scale'])  # number of incomining inhibitory connections

    nest.message(M_INFO, 'build_network', 'Creating excitatory stimulus generator.')

    # Convert synapse weight from mV to pA
    conversion_factor = convert_synapse_weight(
        model_params['tau_m'], model_params['tau_syn_ex'], model_params['C_m'])
    JE_pA = conversion_factor * brunel_params['JE']

    nu_thresh = model_params['V_th'] / (
        CE * model_params['tau_m'] / model_params['C_m'] * JE_pA * np.exp(1.) * tau_syn)
    nu_ext = nu_thresh * brunel_params['eta']

    E_stimulus = nest.Create('poisson_generator', 1, {'rate': nu_ext * CE * 1000.})

    nest.message(M_INFO, 'build_network', 'Creating excitatory spike detector.')

    if params['record_spikes']:
        detector_label = os.path.join(brunel_params['filestem'], 'alpha_' + str(stdp_params['alpha']) + '_spikes')
        E_detector = nest.Create('spike_detector', 1, {'withtime': True, 'to_file': True, 'label': detector_label})

    BuildNodeTime = time.time() - tic

    logger.log(str(BuildNodeTime) + ' # build_time_nodes')
    logger.log(str(memory_thisjob()) + ' # virt_mem_after_nodes')

    tic = time.time()

    nest.SetDefaults('static_synapse_hpc', {'delay': brunel_params['delay']})

    stdp_params['weight'] = JE_pA
    nest.SetDefaults('stdp_pl_synapse_hom_hpc', stdp_params)

    nest.message(M_INFO, 'build_network', 'Connecting stimulus generators.')

    # Connect Poisson generator to neuron

    nest.Connect(E_stimulus, E_neurons, {'rule': 'all_to_all'}, {'model': 'static_synapse_hpc', 'weight': JE_pA})
    nest.Connect(E_stimulus, I_neurons, {'rule': 'all_to_all'}, {'model': 'static_synapse_hpc', 'weight': JE_pA})

    nest.message(M_INFO, 'build_network', 'Connecting excitatory -> excitatory population.')

    nest.Connect(E_neurons, E_neurons,
                 {'rule': 'fixed_indegree', 'indegree': CE, 'autapses': False, 'multapses': True},
                 {'model': 'stdp_pl_synapse_hom_hpc'})

    nest.message(M_INFO, 'build_network', 'Connecting inhibitory -> excitatory population.')

    nest.Connect(I_neurons, E_neurons,
                 {'rule': 'fixed_indegree', 'indegree': CI, 'autapses': False, 'multapses': True},
                 {'model': 'static_synapse_hpc', 'weight': JE_pA * brunel_params['g']})

    nest.message(M_INFO, 'build_network', 'Connecting excitatory -> inhibitory population.')

    nest.Connect(E_neurons, I_neurons,
                 {'rule': 'fixed_indegree', 'indegree': CE, 'autapses': False, 'multapses': True},
                 {'model': 'static_synapse_hpc', 'weight': JE_pA})

    nest.message(M_INFO, 'build_network', 'Connecting inhibitory -> inhibitory population.')

    nest.Connect(I_neurons, I_neurons,
                 {'rule': 'fixed_indegree', 'indegree': CI, 'autapses': False, 'multapses': True},
                 {'model': 'static_synapse_hpc', 'weight': JE_pA * brunel_params['g']})

    if params['record_spikes']:
        local_neurons = list(get_local_nodes(E_neurons))

        if len(local_neurons) < brunel_params['Nrec']:
            nest.message(M_ERROR, 'build_network',
                         '''Spikes can only be recorded from local neurons, but the number of local
                         neurons is smaller than the number of neurons spikes should be recorded from.
                         Aborting the simulation!''')
            exit(1)

        nest.message(M_INFO, 'build_network', 'Connecting spike detectors.')
        nest.Connect(local_neurons[:brunel_params['Nrec']], E_detector)

    # read out time used for building
    BuildEdgeTime = time.time() - tic

    logger.log(str(BuildEdgeTime) + ' # build_edge_time')
    logger.log(str(memory_thisjob()) + ' # virt_mem_after_edges')


def run_simulation():
    # open log file
    logger = Logger(params['log_file'])

    nest.ResetKernel
    nest.set_verbosity(M_INFO)

    logger.log(str(memory_thisjob()) + ' # virt_mem_0')

    build_network(logger)

    tic = time.time()

    nest.Simulate(params['presimtime'])

    PreparationTime = time.time() - tic

    logger.log(str(memory_thisjob()) + ' # virt_mem_after_presim')
    logger.log(str(PreparationTime) + ' # presim_time')

    tic = time.time()

    nest.Simulate(params['simtime'])

    SimCPUTime = time.time() - tic

    logger.log(str(memory_thisjob()) + ' # virt_mem_after_sim')
    logger.log(str(SimCPUTime) + ' # sim_time')

    logger.log(str(compute_rate()) + ' # average rate')

    logger.done()

# ------------------------------------------------------------------------------------


def compute_rate():
    '''compute approximation of average firing rate based on number of
    local nodes, number of local spikes and total time
    '''

    n_local_spikes = nest.GetKernelStatus('local_spike_counter')
    n_local_neurons = nest.GetKernelStatus('network_size') / nest.GetKernelStatus('num_processes')
    simtime = nest.GetKernelStatus('time')
    return 1. * n_local_spikes / (n_local_neurons * simtime) * 1e3

#  ------------------------------------------------------------------------------------


def memory_thisjob():
    '''wrapper to obtain current memory usage'''
    nest.sr('memory_thisjob')
    return nest.spp()

#  ------------------------------------------------------------------------------------


def lambertwm1(x):
    nest.sr('{} LambertWm1'.format(x))
    return nest.spp()

#  ------------------------------------------------------------------------------------


def get_local_nodes(nodes):

    '''
    Generator for efficient looping over local nodes.
    '''

    nvp = nest.GetKernelStatus('total_num_virtual_procs')  # step size

    i = 0
    while i < len(nodes):
        if nest.GetStatus([nodes[i]], 'local')[0]:
            yield nodes[i]
            i += nvp
        else:
            i += 1

#  ------------------------------------------------------------------------------------


class Logger(object):
    '''
    This class defines a logger class used to properly log memory and timing
    information from network simulations. It is used by hpc_benchmark.sli to
    store the information to the log files.
    '''

    def __init__(self, file_name):
        self.max_rank_cout = 5  # copy output to cout for ranks 0..max_rank_cout-1
        self.max_rank_log = 30  # write to log files for ranks 0..max_rank_log-1
        self.line_counter = 0

        if nest.Rank() < self.max_rank_log:

            # convert rank to string, prepend 0 if necessary to make
            # numbers equally wide for all ranks
            rank = '{:0' + str(len(str(self.max_rank_log))) + '}'
            fn = file_name + '_' + rank.format(nest.Rank()) + '.dat'

            self.f = open(fn, 'w')

    def log(self, value):
        if nest.Rank() < self.max_rank_log:
            self.f.write(str(self.line_counter) + ' ' + str(nest.Rank()) + ' ' + value + '\n')
            self.line_counter += 1

        if nest.Rank() < self.max_rank_cout:
            print(str(nest.Rank()) + ' ' + value + '\n', file=sys.stdout)
            print(str(nest.Rank()) + ' ' + value + '\n', file=sys.stderr)

    def done(self):
        if nest.Rank() < self.max_rank_log:
            self.f.close()

# ------------------------------------------------------------------------------------


if __name__ == '__main__':
    run_simulation()
