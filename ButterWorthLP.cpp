//
// Created by yc on 2021/6/28.
//
#include "ButterWorthLP.h"
#include <cmath>


ButterworthLP::ButterworthLP(const double normalizedCutoffFrequency, const size_t filterOrder) :
        _numSOS(0)
{
    if (!coefficients(normalizedCutoffFrequency, filterOrder))
    {
        throw std::domain_error(std::string("Failed to design a FilterData due to invalid parameters (normalized cutoff frequency and / or FilterData order) or instability of the resulting digitalized FilterData."));
    }
}

ButterworthLP::ButterworthLP(const double samplingFrequency, const double cutoffFrequency, const size_t filterOrder) :
        ButterworthLP(cutoffFrequency / samplingFrequency, filterOrder)
{ }

void ButterworthLP::SOS::safeRestoreState(double &z1, double &z2, const bool restore)
{
    if (restore)
    {
        _z1 = z1;
        _z2 = z2;
    }
    else
    {
        z1 = _z1;
        z2 = _z2;
    }
}

void ButterworthLP::SOS::stepInitialization(const double value)
{
    // Set second-order section state to steady state w.r.t. given step response value
    // http://www.emt.tugraz.at/publications/diplomarbeiten/da_hoebenreich/node21.html
    _z1 = _z2 = value / (1.0 + _a1 + _a2);
}

double ButterworthLP::SOS::process(const double input)
{
    /**
    SOS state space model
    z' = A * z + b * x
    y = c^T * z + d * x

    with A = [-a1  -a2; 1     0]   b = [1; 0]   c = [b1 - b0*a1; b2 - b0*a2]   d = [b0]
    */

    double x = input;
    double y = x;
    double z1_new, z2_new;

    z1_new = -_a1 * _z1 - _a2 * _z2 + x;
    z2_new = _z1;
    y = _preCompStateSpaceOutputVec1 * _z1 + _preCompStateSpaceOutputVec2 * _z2 + _b0 * x;
    _z1 = z1_new;
    _z2 = z2_new;

    // Include SOS gain factor
    y *= _gain;

    return y;
}

void ButterworthLP::addSOS(const SOS sos)
{
    _sosVec.push_back(sos);
    ++_numSOS;
}

void ButterworthLP::stepInitialization(const double value)
{
    double stepResponseValue = value;

    // Propagate step initialization through all second-order sections
    for (size_t i = 0; i < _numSOS; ++i)
    {
        _sosVec[i].stepInitialization(stepResponseValue);
        stepResponseValue = _sosVec[i].process(stepResponseValue);
    }
}

double ButterworthLP::process(const double input)
{
    double x = input;
    double y = x;

    // Cascade all second-order sections s.t. output_1 of SOS i is input_1 for SOS i+1
    for (size_t i = 0; i < _numSOS; ++i)
    {
        y = _sosVec[i].process(x);
        x = y;
    }

    return y;
}

void ButterworthLP::filter(const double *input, double *output, const size_t size, const double initialConditionValue, const bool forwardBackward)
{
    // Save all current SOS states before offline filtering
    std::vector<double> zSaved(_numSOS * 2);
    for (size_t i = 0; i < _numSOS; ++i)
    {
        _sosVec[i].safeRestoreState(zSaved[i], zSaved[i + 1], false);
    }

    // Set initial step response conditions
    stepInitialization(initialConditionValue);

    // Filtering on input_1 data
    for (size_t i = 0; i < size; ++i)
    {
        output[i] = process(input[i]);
    }

    // Additional backward filtering on filtered output_1 data if requested
    if (forwardBackward)
    {
        for (size_t i = size; i > 0;)
        {
            output[--i] = process(output[i]);
        }
    }

    // Restore all SOS states
    for (size_t i = 0; i < _numSOS; ++i)
    {
        _sosVec[i].safeRestoreState(zSaved[i], zSaved[i + 1], true);
    }
}

bool ButterworthLP::coefficients(const double normalizedCutoffFrequency, const size_t filterOrder)
{
    // Assure valid parameters
    if (filterOrder < 1 || normalizedCutoffFrequency <= 0 || normalizedCutoffFrequency > 1)
    {
        return false;
    }

    std::vector<std::complex<double>> poles(filterOrder);

    // Prewarp the analog prototype's cutoff frequency
    double omegaCutoff = 2 * tan(M_PI * normalizedCutoffFrequency);

    double gain = pow(omegaCutoff, filterOrder);
    double initialGain = gain;

    std::complex<double> two(2.0, 0);

    for (size_t i = 0, i_end = (filterOrder + 1) / 2; i < i_end; ++i)
    {
        size_t i2 = 2 * i;

        /**
        Design the analog prototype Butterworth lowpass FilterData
        */

        // Generate s-poles of prototype FilterData
        double phi = (double)(i2 + 1) * M_PI / (2 * filterOrder);
        double real = -sin(phi);
        double imag = cos(phi);

        std::complex<double> pole = std::complex<double>(real, imag);

        /**
        Customize analog prototype FilterData w.r.t cutoff frequency
        */

        // Scale s-pole with the cutoff frequency
        pole *= omegaCutoff;

        /**
        Digitalize the analog FilterData
        */

        // Map pole from s-plane to z-plane using bilinear transform
        std::complex<double> s = pole;
        pole = (two + s) / (two - s);

        // Update overall gain in respect of z-pole gain
        gain *= abs((two - s));

        // Ensure z-pole lies in unit circle of z-plane
        if (abs(pole) > 1)
        {
            return false;
        }

        // Add stable z-pole
        poles[i2] = pole;

        // Odd FilterData order: ignore the second complex conjugate pole
        if (i2 + 1 >= filterOrder)
        {
            break;
        }

        // Do the same as above with the conjugate complex pole
        pole = std::complex<double>(real, -imag);
        pole *= omegaCutoff;
        s = pole;
        pole = (two + s) / (two - s);
        gain *= abs((two - s));
        if (abs(pole) > 1)
        {
            return false;
        }
        poles[i2 + 1] = pole;
    }

    // Distribute the overall gain over all z-poles
    double overallGain = initialGain * (initialGain / gain);
    double distributedPoleGain = pow(overallGain, 1.0 / (double)filterOrder);
    double distributedPolePairGain = distributedPoleGain * distributedPoleGain;

    /**
    Generate second-order sections from conjugate complex z-pole pairs
    */

    for (size_t i = 0, i_end = filterOrder - 1; i < i_end; i += 2)
    {
        addSOS(SOS(1.0, 2.0, 1.0, -(poles[i] + poles[i + 1]).real(), (poles[i] * poles[i + 1]).real(), distributedPolePairGain));
    }

    // Odd FilterData order: remaining single z-pole requires additional second-order section
    if (filterOrder % 2 == 1)
    {
        addSOS(SOS(1.0, 1.0, 0.0, -poles[filterOrder - 1].real(), 0.0, distributedPoleGain));
    }

    return true;
}

