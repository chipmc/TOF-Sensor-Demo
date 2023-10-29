// People Counter Class
// Author: Chip McClelland
// Date: May 2023
// License: GPL3
// In this class, we look at the occpancy values and determine what the occupancy count should be 
// Note, this code is limited to a single sensor with two zones

#ifndef __PEOPLECOUNTER_H
#define __PEOPLECOUNTER_H

#include "Particle.h"
#include "PeopleCounterConfig.h"

/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * peopleCounter::instance().setup();
 * 
 * From global application loop you must call:
 * peopleCounter::instance().loop();
 */
class PeopleCounter {
public:
    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use peopleCounter::instance() to instantiate the singleton.
     */
    static PeopleCounter &instance();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use peopleCounter::instance().setup();
     */
    void setup();

    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use peopleCounter::instance().loop();
     */
    void loop();

    int getCount();
    int getLimit();
    void setCount(int value);
    void setLimit(int value);

protected:
    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use peopleCounter::instance() to instantiate the singleton.
     */
    PeopleCounter();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~PeopleCounter();

    /**
     * This class is a singleton and cannot be copied
     */
    PeopleCounter(const PeopleCounter&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    PeopleCounter& operator=(const PeopleCounter&) = delete;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static PeopleCounter *_instance;

    /**
     * @brief This function simplifies the display and supports testing in the field.
     * 
     * The display options are set in the PeopleCounterConfig file as the TENFOOTDISPLAY declation
    */
    void printBigNumbers(int number);

    /**
     * @brief Algorithm that makes sense of nonsensical state transitions by filling in the
     *        gaps to help the stack make sense again. 
    */
    void applyMagicalStateMapCorrections(int newOccupancyState);

    int count = 0;
    int limit = DEFAULT_PEOPLE_LIMIT;
};
#endif  /* __PEOPLECOUNTER_H */
