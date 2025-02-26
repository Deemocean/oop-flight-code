#include "RockblockCommand.hpp"
#include <map>
#include <stdint.h>

#ifndef _FAULT_HPP_
#define _FAULT_HPP_

/**
 * @brief maps opcodes to states of all faults
 *
 */
class FaultInterface
{
private:
    bool suppressed;
    bool forced;
    bool signaled;
    bool base;
    uint16_t opcode;

public:
    /**
     * @brief The Opcode Lookup map for faults.
     *
     */
    static std::map<uint16_t, FaultInterface *> opcode_lookup;

    virtual void signal();
    virtual void release();
    virtual void force();
    virtual void suppress();
    virtual void restore();
    virtual bool get_base();
    virtual bool get_signaled();
    virtual bool get_suppressed();
};

/**
 * @brief Represents a fault's current state.
 *
 */
class Fault : public FaultInterface
{
private:
    bool suppressed;
    bool forced;
    bool signaled;
    bool base;
    uint16_t opcode;

public:
    /**
     * @brief Construct a new Fault object and adds it to the Fault registry using the opcode.
     * Sets all initial states to false.
     *
     * @param opcode The command opcode corresponding to the fault
     */
    Fault(uint16_t opcode);

    /**
     * @brief Signals the fault, setting the base flag to high if the fault is not suppressed.
     *
     */
    void signal();

    /**
     * @brief Releases the fault's signal, which makes the base flag low.
     *
     */
    void release();

    /**
     * @brief Force the fault, overwriting the current base flag with high, no matter the signal.
     *
     * Unsuppresses the fault if it was suppressed.
     */
    void force();

    /**
     * @brief Suppresses the fault, overwriting the current base flag with low, no matter the signal.
     *
     * Unforces the fault if it was forced.
     */
    void suppress();

    /**
     * @brief Restore the fault base to the raw sensor data. Unsuppresses and unforces the fault.
     */
    void restore();

    /**
     * @brief The current value of the fault
     *
     * @return true when the fault is being signaled and the fault is not suppressed
     * @return false when the fault is not being signaled or the fault is being suppressed
     */
    bool get_base();

    /**
     * @brief Whether the fault is being signalled.
     *
     * @return true when the fault is being signaled high (irregardless of suppression status)
     * @return false when the fault is being signaled low (irregardless of suppression status)
     */
    bool get_signaled();

    /**
     * @brief Whether the fault is currently being forced.
     *
     * @return true when the fault is being forced.
     * @return false when the fault is not being forced.
     */
    bool get_forced();

    /**
     * @brief Whether the fault is currently being suppressed.
     *
     * @return true when the fault is being suppressed.
     * @return false when the fault is not being suppressed
     */
    bool get_suppressed();

    /**
     * @brief Serializes the four main control booleans.
     *
     * Bit 3 - Base (B),
     * Bit 2 - Forced (F)
     * Bit 1 - suppressed (Su),
     * Bit 0 - Signaled (Si)
     *
     * @return uint8_t - The four major indicators of fault status. 0000BFSuSi
     */
    uint8_t serialize();
};
#endif