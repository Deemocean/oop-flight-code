#include <cstring>
#include <limits>
#include <map>
#include <stdint.h>
#include <type_traits>
#include <vector>

#ifndef _SFRFIELD_HPP_
#define _SFRFIELD_HPP_

class SFRInterface
{
private:
    int field_value;    // The value of the field cast to an int
    int data_type;      // An int representing the field's type T, 1 for bool, 2 for uint8_t, 3 for uint16_t, and 4 for uint64_t
    int address_offset; // This field's byte offset from the beginning of the EEPROM section where SFR data is currently stored
                        // sfr_address + address_offset gives this field's location in EEPROM
    bool restore;       // If the field should be restored or not

public:
    static std::map<int, SFRInterface *> opcode_lookup; // </brief Op Code Lookup Map For SFR Field Uplink Override
    // static std::vector<SFRInterface *> sfr_fields_vector;
    static std::map<int, int> int_map;

#ifdef DEBUG
    static void resetSFR()
    {
        for (auto const &kv : SFRInterface::opcode_lookup) {
            opcode_lookup[kv.first]->reset();
        }
    }

    virtual void reset();
#endif

    virtual ~SFRInterface(){};
    static void setFieldVal(int opcode, uint32_t arg1);
    virtual void setValue(uint32_t arg1);

    // For setting the SFRInterface members for EEPROM saving and restoring
    void setFieldValue(int val);
    int getFieldValue();

    void setDataType(int type);
    int getDataType();

    void setAddressOffset(int offset);
    int getAddressOffset();

    void setRestore(bool res);
    bool getRestore();
};

template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
class SFRField : public SFRInterface
{
private:
    T value;      // </brief Field Value, Only Arithmetic Types Allowed
    T max;        // </brief Inclusive Minium Value
    T min;        // </brief Inclusive Maximum Value
    bool bounded; // </brief If max and min are bounded beyond data type
    int opcode;   // </brief Uplink Op Code to set this field
    int resolution;

#ifdef DEBUG
    T inital;
#endif

public:
    SFRField(T default_val, T min, T max, int opcode_val, int address_offset, bool restore)
    {
        value = default_val;
        min = min;
        max = max;
        bounded = true;
        opcode = opcode_val;
        resolution = 1;
        SFRInterface::setFieldValue((int)value);
        if (sizeof(T) == sizeof(uint32_t))
            SFRInterface::setDataType(4);
        else if (sizeof(T) == sizeof(uint16_t))
            SFRInterface::setDataType(3);
        else if (sizeof(T) == sizeof(uint8_t))
            SFRInterface::setDataType(2);
        else if (sizeof(T) == sizeof(bool))
            SFRInterface::setDataType(1);
        SFRInterface::setAddressOffset(address_offset);
        SFRInterface::setRestore(restore);
#ifdef DEBUG
        T inital = default_val;
#endif
        SFRInterface::opcode_lookup[opcode_val] = this;
        SFRInterface::int_map[0] = 0;
        // SFRInterface::sfr_fields_vector.push_back(this);
    }

    SFRField(T default_val, int opcode_val, int address_offset, bool restore)
    {
        value = default_val;
        bounded = false;
        opcode = opcode_val;
        resolution = 1;
        SFRInterface::setFieldValue((int)value);
        if (sizeof(T) == sizeof(uint32_t))
            SFRInterface::setDataType(4);
        else if (sizeof(T) == sizeof(uint16_t))
            SFRInterface::setDataType(3);
        else if (sizeof(T) == sizeof(uint8_t))
            SFRInterface::setDataType(2);
        else if (sizeof(T) == sizeof(bool))
            SFRInterface::setDataType(1);
        SFRInterface::setAddressOffset(address_offset);
        SFRInterface::setRestore(restore);

#ifdef DEBUG
        T inital = default_val;
#endif
        SFRInterface::opcode_lookup[opcode_val] = this;
        // SFRInterface::int_map[0] = 0;
        // SFRInterface::sfr_fields_vector.push_back(this);
    }

    SFRField(float default_val, float min, float max, int opcode_val, float resolution, int address_offset, bool restore)
    {
        value = default_val * resolution;
        min = min;
        max = max;
        bounded = true;
        opcode = opcode_val;
        resolution = resolution;
        SFRInterface::setFieldValue((int)value);
        if (sizeof(T) == sizeof(uint32_t))
            SFRInterface::setDataType(4);
        else if (sizeof(T) == sizeof(uint16_t))
            SFRInterface::setDataType(3);
        else if (sizeof(T) == sizeof(uint8_t))
            SFRInterface::setDataType(2);
        else if (sizeof(T) == sizeof(bool))
            SFRInterface::setDataType(1);
        SFRInterface::setAddressOffset(address_offset);
        SFRInterface::setRestore(restore);

#ifdef DEBUG
        T inital = default_val;
#endif
        SFRInterface::opcode_lookup[opcode_val] = this;
        // SFRInterface::int_map[0] = 0;
        // SFRInterface::sfr_fields_vector.push_back(this);
    }

    operator T()
    {
        return value;
    }

    T get()
    {
        return value;
    }

    float get_float()
    {
        return value / resolution;
    }

    void set(T input)
    {
        if ((bounded && input <= max && input >= min) || (!bounded)) {
            value = input;
        }
    }

#ifdef DEBUG
    void reset()
    {
        value = initial;
    }
#endif
    void setValue(uint32_t arg1)
    {
        // Convert 32bit word into Target Type
        // TODO Joining Two uint16s FS-158
        static_assert(sizeof(T) <= sizeof arg1, "Templated Type is larger than input.");
        T casted;
        std::memcpy(&casted, &arg1, sizeof(T));
        set(casted);
    }

    // Postfix increment operator.
    void operator++(int)
    {
        set(value + 1);
    }

    // Assignment Operator
    void operator=(T val)
    {
        set(val);
    }

    void operator+=(T val)
    {
        set(value + val);
    }
};

#endif