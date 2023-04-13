
// SPDX-License-Identifier: MIT
// Copyright (c) 2023 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_COMMON_BLE

#include <assert.h>
#include <string.h>

#include <pbdrv/bluetooth.h>

#include "py/obj.h"
#include "py/misc.h"
#include "py/runtime.h"

#include <pybricks/common.h>
#include <pybricks/util_pb/pb_error.h>
#include <pybricks/util_pb/pb_task.h>

// The code currently passes integers and floats directly as bytes so requires
// little-endian to get the correct ordering over the air.
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "this module requires little endian processor"
#endif

#define OBSERVED_DATA_MAX_SIZE (31 /* max adv data size */ - 5 /* overhead */)

typedef struct {
    int8_t rssi;
    uint8_t size;
    uint8_t data[OBSERVED_DATA_MAX_SIZE];
} observed_data_t;

// not to be confused with number of channels, which is one more than this
#define MAX_CHANNEL_NUMBER 15

// pointer to dynamically allocated memory - needed for driver callback
static observed_data_t *observed_data;
static uint8_t num_observed_data;

typedef struct {
    mp_obj_base_t base;
    uint8_t broadcast_channel;
    observed_data_t observed_data[];
} pb_obj_BLE_t;

/**
 * Type codes used for encoding/decoding data.
 */
typedef enum {
    // NB: These values are sent over the air so the numeric values must not be changed.
    // There can be at most 8 types since the values have to fit in 3 bits.

    /** The Python @c None value. */
    PB_BLE_BROADCAST_DATA_TYPE_NONE = 0,
    /** The Python @c True value. */
    PB_BLE_BROADCAST_DATA_TYPE_TRUE = 1,
    /** The Python @c False value. */
    PB_BLE_BROADCAST_DATA_TYPE_FALSE = 2,
    /** The Python @c int type. */
    PB_BLE_BROADCAST_DATA_TYPE_INT = 3,
    /** The Python @c float type. */
    PB_BLE_BROADCAST_DATA_TYPE_FLOAT = 4,
    /** The Python @c str type. */
    PB_BLE_BROADCAST_DATA_TYPE_STR = 5,
    /** The Python @c bytes type. */
    PB_BLE_BROADCAST_DATA_TYPE_BYTES = 6,
} pb_ble_broadcast_data_type_t;

#define MFG_SPECIFIC 0xFF
#define LEGO_CID 0x0397

STATIC void handle_observe_event(pbdrv_bluetooth_ad_type_t event_type, const uint8_t *data, uint8_t length, int8_t rssi) {
    if (event_type == PBDRV_BLUETOOTH_AD_TYPE_ADV_NONCONN_IND && length >= 5 && data[1] == MFG_SPECIFIC && pbio_get_uint16_le(&data[2]) == LEGO_CID) {
        uint8_t channel = data[4];

        if (channel >= num_observed_data) {
            // ignore out of range channels
            return;
        }

        observed_data[channel].rssi = rssi;
        observed_data[channel].size = data[0] - 4;
        memcpy(observed_data[channel].data, &data[5], OBSERVED_DATA_MAX_SIZE);
    }
}

/**
 * Appends the value of a Python object to the advertising data.
 *
 * @param [in]  dst     Pointer to the start of the manufacturer-specific advertising data.
 * @param [in]  index   The index in @p dst where the value should be written.
 * @param [in]  src     The value to write.
 * @param [in]  size    The size of @p src in bytes.
 * @param [in]  type    The data type of @p src.
 * @returns             The next free index in @p dst after adding the new data.
 * @throws ValueError   If data exceeds available space remaining in @p dst.
 */
STATIC size_t pb_module_ble_append(uint8_t *dst, size_t index, const void *src, size_t size, pb_ble_broadcast_data_type_t type) {
    size_t next_index = index + size + 1;

    if (next_index > OBSERVED_DATA_MAX_SIZE) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload limited to 26 bytes"));
    }

    dst[index] = type << 5 | size;
    memcpy(&dst[index + 1], src, size);

    return next_index;
}

/**
 * Encodes a Python object using the Pybricks Broadcast encoding scheme and
 * appends it to the advertising data.
 *
 * @p arg must be @c None, @c True, @c False, an @c int, a @c float, a @c str
 * or bytes-like (supports buffer protocol).
 *
 * @param [in]  dst     Pointer to the start of the manufacturer-specific advertising data.
 * @param [in]  index   The index in @p dst where the value should be written.
 * @param [in]  arg     The Python object to be encoded.
 * @returns             The next free index in @p dst after adding the new data.
 * @throws ValueError   If data exceeds available space remaining in @p dst.
 * @throws TypeError    If @p arg is not one of the supported types.
 */
STATIC size_t pb_module_ble_encode(void *dst, size_t index, mp_obj_t arg) {
    if (arg == mp_const_none) {
        return pb_module_ble_append(dst, index, NULL, 0, PB_BLE_BROADCAST_DATA_TYPE_NONE);
    }

    if (arg == mp_const_true) {
        return pb_module_ble_append(dst, index, NULL, 0, PB_BLE_BROADCAST_DATA_TYPE_TRUE);
    }

    if (arg == mp_const_false) {
        return pb_module_ble_append(dst, index, NULL, 0, PB_BLE_BROADCAST_DATA_TYPE_FALSE);
    }

    mp_int_t int_value;
    if (mp_obj_get_int_maybe(arg, &int_value)) {
        if (int_value >= INT8_MIN && int_value <= INT8_MAX) {
            int8_t int8_value = int_value;
            return pb_module_ble_append(dst, index, &int8_value, sizeof(int8_value), PB_BLE_BROADCAST_DATA_TYPE_INT);
        }

        if (int_value >= INT16_MIN && int_value <= INT16_MAX) {
            int16_t int16_value = int_value;
            return pb_module_ble_append(dst, index, &int16_value, sizeof(int16_value), PB_BLE_BROADCAST_DATA_TYPE_INT);
        }

        #if __SIZEOF_POINTER__ == 4
        return pb_module_ble_append(dst, index, &int_value, sizeof(int_value), PB_BLE_BROADCAST_DATA_TYPE_INT);
        #else
        if (int_value >= INT32_MIN && int_value <= INT32_MAX) {
            int32_t int32_value = int_value;
            return pb_module_ble_append(dst, index, &int32_value, sizeof(int32_value), PB_BLE_BROADCAST_DATA_TYPE_INT);
        }

        mp_raise_msg(&mp_type_OverflowError, MP_ERROR_TEXT("integers are limited to 32 bits"));
        #endif
    }

    #if MICROPY_FLOAT_IMPL != MICROPY_FLOAT_IMPL_NONE
    mp_float_t float_value;
    if (mp_obj_get_float_maybe(arg, &float_value)) {
        #if MICROPY_FLOAT_IMPL == MICROPY_FLOAT_IMPL_DOUBLE
        float single_value = float_value;
        return pb_module_ble_append(dst, index, &single_value, sizeof(single_value), PB_BLE_BROADCAST_DATA_TYPE_FLOAT);
        #elif MICROPY_FLOAT_IMPL == MICROPY_FLOAT_IMPL_FLOAT
        return pb_module_ble_append(dst, index, &float_value, sizeof(float_value), PB_BLE_BROADCAST_DATA_TYPE_FLOAT);
        #else
        #error "unsupported MICROPY_FLOAT_IMPL"
        #endif
    }
    #endif

    mp_buffer_info_t info;
    if (mp_get_buffer(arg, &info, MP_BUFFER_READ)) {
        // REVISIT: possible upstream contribution - add str type to info.typecode
        bool is_str = mp_obj_is_str(arg);
        return pb_module_ble_append(dst, index, info.buf, info.len, is_str ? PB_BLE_BROADCAST_DATA_TYPE_STR : PB_BLE_BROADCAST_DATA_TYPE_BYTES);
    }

    mp_raise_TypeError(MP_ERROR_TEXT("must be None, True, False, int, float, str or bytes"));

    MP_UNREACHABLE
}

/**
 * Sets the broadcast advertising data and enables broadcasting on the Bluetooth
 * radio if it is not already enabled.
 *
 * The first argument is the "channel" and the remaining arguments are encoded
 * in the advertising data.
 *
 * @param [in]  n_args  The number of args.
 * @param [in]  args    The args passed in Python code.
 * @throws ValueError   If the channel is out of range or the encoded arguments
 *                      exceeded the available space.
 * @throws TypeError    If any of the arguments are of a type that can't be encoded.
 */
STATIC mp_obj_t pb_module_ble_broadcast(size_t n_args, const mp_obj_t *args) {
    // REVISIT: disable this method on move hub and city hub?
    // This method will raise an OSError on move hub if it is called while the
    // move hub is connected to Pybricks Code. Also, broadcasting interferes
    // with observing even when not connected to Pybricks Code. On the city
    // hub, this method succeeds, but nothing is actually sent over the air.

    pb_obj_BLE_t *self = MP_OBJ_TO_PTR(args[0]);

    struct {
        pbdrv_bluetooth_value_t v;
        uint8_t d[5 + OBSERVED_DATA_MAX_SIZE];
    } value;

    size_t index = 0;
    for (size_t i = 1; i < n_args; i++) {
        index = pb_module_ble_encode(&value.v.data[5], index, args[i]);
    }

    value.v.size = index + 5;
    value.v.data[0] = index + 4; // length
    value.v.data[1] = MFG_SPECIFIC;
    pbio_set_uint16_le(&value.v.data[2], LEGO_CID);
    value.v.data[4] = self->broadcast_channel;

    pbio_task_t task;
    pbdrv_bluetooth_start_broadcasting(&task, &value.v);

    pb_wait_task(&task, -1);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR(pb_module_ble_broadcast_obj, 1, pb_module_ble_broadcast);

/**
 * Decodes data that was received by the Bluetooth radio.
 *
 * @param [in]      data    Pointer to the start of the advertising data.
 * @param [in,out]  index   When calling, set to the index in @p data to read.
 *                          On return, the value is updated to the next index.
 * @returns                 The decoded value as a Python object.
 * @throws RuntimeError     If the data was invalid and could not be decoded.
 */
STATIC mp_obj_t pb_module_ble_decode(observed_data_t *data, size_t *index) {
    uint8_t size = data->data[*index] & 0x1F;
    pb_ble_broadcast_data_type_t data_type = data->data[*index] >> 5;

    (*index)++;

    switch (data_type) {
        case PB_BLE_BROADCAST_DATA_TYPE_NONE:
            assert(size == 0);
            return mp_const_none;
        case PB_BLE_BROADCAST_DATA_TYPE_TRUE:
            assert(size == 0);
            return mp_const_true;
        case PB_BLE_BROADCAST_DATA_TYPE_FALSE:
            assert(size == 0);
            return mp_const_false;
        case PB_BLE_BROADCAST_DATA_TYPE_INT:
            if (size == sizeof(int8_t)) {
                int8_t int8_value = data->data[*index];
                (*index) += sizeof(int8_value);
                return MP_OBJ_NEW_SMALL_INT(int8_value);
            }

            if (size == sizeof(int16_t)) {
                int16_t int16_value = pbio_get_uint16_le(&data->data[*index]);
                (*index) += sizeof(int16_value);
                return MP_OBJ_NEW_SMALL_INT(int16_value);
            }

            if (size == sizeof(int32_t)) {
                int32_t int32_value = pbio_get_uint32_le(&data->data[*index]);
                (*index) += sizeof(int32_value);
                return mp_obj_new_int(int32_value);
            }

            break;

        case PB_BLE_BROADCAST_DATA_TYPE_FLOAT:
            #if MICROPY_FLOAT_IMPL != MICROPY_FLOAT_IMPL_NONE
        {
            union {
                float f;
                uint32_t u;
            } float_value;
            float_value.u = pbio_get_uint32_le(&data->data[*index]);
            (*index) += sizeof(float_value);
            return mp_obj_new_float_from_f(float_value.f);
        }
            #else
            break;
            #endif

        case PB_BLE_BROADCAST_DATA_TYPE_STR: {
            const char *str_data = (void *)&data->data[*index];
            (*index) += size;
            return mp_obj_new_str(str_data, size);
        }

        case PB_BLE_BROADCAST_DATA_TYPE_BYTES: {
            const byte *bytes_data = (void *)&data->data[*index];
            (*index) += size;
            return mp_obj_new_bytes(bytes_data, size);
        }
    }

    mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("received bad data"));
}

/**
 * Retrieves the last received advertising data and enables observing in the
 * Bluetooth radio if it is not already enabled.
 *
 * @param [in]  self_in     The BLE object.
 * @param [in]  channel_in  Python object containing the channel number.
 * @returns                 Python object containing a tuple of the RSSI and
 *                          the decoded data.
 * @throws ValueError       If the channel is out of range.
 * @throws RuntimeError     If the last received data was invalid.
 */
STATIC mp_obj_t pb_module_ble_observe(mp_obj_t self_in, mp_obj_t channel_in) {
    mp_int_t channel = mp_obj_get_int_truncated(channel_in);

    if (channel < 0 || channel >= num_observed_data) {
        pb_assert(PBIO_ERROR_INVALID_ARG);
    }

    pbio_task_t task;
    pbdrv_bluetooth_start_observing(&task, handle_observe_event);
    pb_wait_task(&task, -1);

    observed_data_t *ch_data = &observed_data[channel];

    // Objects can be encoded in as little as one byte so we could have up to
    // this many objects received.
    mp_obj_t items[OBSERVED_DATA_MAX_SIZE];

    size_t index = 0;
    size_t i;
    for (i = 0; i < OBSERVED_DATA_MAX_SIZE; i++) {
        if (index >= ch_data->size) {
            break;
        }

        items[i] = pb_module_ble_decode(ch_data, &index);
    }

    mp_obj_t result[2];
    result[0] = MP_OBJ_NEW_SMALL_INT(ch_data->rssi); // RSSI
    result[1] = mp_obj_new_tuple(i, items); // data

    return mp_obj_new_tuple(MP_ARRAY_SIZE(result), result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pb_module_ble_observe_obj, pb_module_ble_observe);

STATIC mp_obj_t pb_module_ble_version(mp_obj_t self_in) {
    const char *version = pbdrv_bluetooth_get_fw_version();
    return mp_obj_new_str(version, strlen(version));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pb_module_ble_version_obj, pb_module_ble_version);

STATIC const mp_rom_map_elem_t common_BLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_broadcast), MP_ROM_PTR(&pb_module_ble_broadcast_obj) },
    { MP_ROM_QSTR(MP_QSTR_observe), MP_ROM_PTR(&pb_module_ble_observe_obj) },
    { MP_ROM_QSTR(MP_QSTR_version), MP_ROM_PTR(&pb_module_ble_version_obj) },
};
STATIC MP_DEFINE_CONST_DICT(common_BLE_locals_dict, common_BLE_locals_dict_table);

static const mp_obj_type_t pb_type_BLE = {
    .base = { &mp_type_type },
    .name = MP_QSTR_BLE,
    .locals_dict = (mp_obj_dict_t *)&common_BLE_locals_dict,
};

/**
 * Creates a new (singleton) instance of the BLE class.
 *
 * Do not call this function more than once unless pb_type_BLE_cleanup() is called first.
 *
 * @param [in]  broadcast_channel_in    (int) The channel number to use for broadcasting.
 * @param [in]  last_observe_channel_in  (int) The highest channel number to observe.
 *                                      All lower channel numbers will also be observed.
 * @throws ValueError                   If either of the parameters is less than 0 or
 *                                      greater than ::MAX_CHANNEL_NUMBER.
 */
mp_obj_t pb_type_BLE_new(mp_obj_t broadcast_channel_in, mp_obj_t last_observe_channel_in) {
    // making the assumption that this is only called once before each pb_type_BLE_cleanup()
    assert(observed_data == NULL);

    mp_int_t broadcast_channel = mp_obj_get_int(broadcast_channel_in);

    if (broadcast_channel < 0 || broadcast_channel > MAX_CHANNEL_NUMBER) {
        mp_raise_ValueError(MP_ERROR_TEXT("broadcast channel must be 0 to " MP_STRINGIFY(MAX_CHANNEL_NUMBER)));
    }

    mp_int_t last_observe_channel = mp_obj_get_int(last_observe_channel_in);

    if (last_observe_channel < 0 || last_observe_channel > MAX_CHANNEL_NUMBER) {
        mp_raise_ValueError(MP_ERROR_TEXT("max observe channel must be 0 to " MP_STRINGIFY(MAX_CHANNEL_NUMBER)));
    }

    pb_obj_BLE_t *self = mp_obj_malloc_var(pb_obj_BLE_t, observed_data_t, last_observe_channel, &pb_type_BLE);
    self->broadcast_channel = broadcast_channel;

    // globals for driver callback
    observed_data = self->observed_data;
    num_observed_data = last_observe_channel + 1;

    return MP_OBJ_FROM_PTR(self);
}

void pb_type_BLE_cleanup(void) {
    pbdrv_bluetooth_stop_broadcasting();
    pbdrv_bluetooth_stop_observing();
    observed_data = NULL;
    num_observed_data = 0;
    // TODO: wait for stop?
}

#endif // PYBRICKS_PY_COMMON_BLE
