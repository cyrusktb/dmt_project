#ifndef FSR_H
#define FSR_H

#include "mbed.h"

/** Force sensitive resistor class using an AnalogIn pin
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "FSR.h"
 * FSR fsr(p20, 10); // Pin 20 is used as the AnalogIn pin and a 10k resistor is used as a voltage divider
 * int main(){
 *     while (1)
 *     {
 *         printf("The raw data is %f\n", fsr.readRaw());
 *         printf("The resistance of the FSR is %f\n", fsr.readFSRResistance());
 *         printf("The weight on the FSR is %f\n\n", fsr.readWeight());
 *         wait(0.3); //just here to slow down the output for easier reading
 *     }
 * }
 * @endcode
 */

class FSR
{
public:
    /** Create an FSR object
     *
     * @param Pin AnalogIn pin number
     * @param resistance resistance of the voltage divider resistor in k
     */
    FSR(PinName Pin, float resistance);
    
    /** Read the raw data
     *
     * @return the raw float data ranging from 0 to 1
     */
    float readRaw();
    
    /** Read the resistance of the FSR
     *
     * @return the resistance of the FSR
     */
    float readFSRResistance();   
    
    /** Read the weight in N. 0 anyway if the weight is less than 100g
     *
     * @return the weight ranging from 100g to 10000g
     */
     float readWeight();
protected:
    AnalogIn _ain;
    float _r;
};

#endif