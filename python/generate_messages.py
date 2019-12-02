#!/usr/bin/env python3

"""Called by make.py."""

import argparse
import os
import re


def write_msg_header(
    output_dir, msg_name, member_var_types, constructor_arg_types, member_var_names
):
    """Write message header file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_name -- name of message in camel case
    member_var_types -- list of member variable types
    constructor_arg_types -- list of function argument types
    member_var_names -- list of member variable names
    """
    with open(f"{msg_name}Packet.hpp", "w") as output:
        output.write(
            """#pragma once

#include <wpi/StringRef.h>

#include <string>

#include "communications/PacketType.hpp"
#include "dsdisplay/Packet.hpp"

namespace frc3512 {

"""
        )
        output.write(f"class {msg_name}Packet {{\n")
        output.write("public:\n")
        output.write(f"    int8_t ID = static_cast<int8_t>(PacketType::k{msg_name});\n")

        default_vals = {"double": " = 0.0;\n", "int": " = 0;\n", "bool": " = false;\n"}
        for i in range(len(member_var_types)):
            output.write(f"    {member_var_types[i]} {member_var_names[i]}")
            try:
                output.write(default_vals[member_var_types[i]])
            except KeyError:
                # No known default value, so use default constructor instead
                output.write(";\n")
        output.write(
            f"""
    {msg_name}Packet() = default;

    /**
     * Construct a {msg_name}Packet with the given fields.
     */
    {msg_name}Packet("""
        )
        output.write(
            ", ".join(
                [
                    x[0] + " " + x[1]
                    for x in zip(constructor_arg_types, member_var_names)
                ]
            )
        )
        output.write(
            f""");

    /**
     * Deserializes the given packet
     *
     * @param packet The packet to deserialize
     */
    {msg_name}Packet(Packet& packet);

    /**
     * Serializes the given packet.
     *
     * The contents of the packet should be passed to whatever communication
     * layer that takes a raw buffer
     */
    Packet Serialize() const;

    /**
     * Deserializes the given packet.
     *
     * @param packet The buffer containing the packet.
     * @param size   The length of the packet.
     */
    void Deserialize(const char* buf, size_t size);

    /**
     * Deserializes the given packet.
     *
     * @param packet The packet to deserialize.
     */
    void Deserialize(Packet& packet);
}};

}}"""
        )
    os.rename(
        f"{msg_name}Packet.hpp",
        f"{output_dir}/include/communications/{msg_name}Packet.hpp",
    )


def write_msg_source(
    output_dir, msg_name, constructor_arg_types, member_var_names, serial_names
):
    """Write message source file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_name -- name of message in camel case
    constructor_arg_types -- list of function argument types
    member_var_names -- list of member variable names
    serial_names -- list of member variable names to serialize/deserialize
    """
    with open(f"{msg_name}Packet.cpp", "w") as output:
        output.write(
            f"""#include "communications/{msg_name}Packet.hpp"

using namespace frc3512;

"""
        )
        output.write(f"{msg_name}Packet::{msg_name}Packet(")
        output.write(
            ", ".join(
                [
                    x[0] + " " + x[1]
                    for x in zip(constructor_arg_types, member_var_names)
                ]
            )
        )
        output.write(") {\n")
        for name in member_var_names:
            output.write(f"    this->{name} = {name};\n")
        output.write(
            f"""}}

{msg_name}Packet::{msg_name}Packet(Packet& packet) {{
    Deserialize(packet);
}}

Packet {msg_name}Packet::Serialize() const {{
    Packet packet;
"""
        )
        for name in serial_names:
            output.write(f"    packet << {name};\n")
        output.write(
            f"""    return packet;
}}

void {msg_name}Packet::Deserialize(Packet& packet) {{
"""
        )
        for name in serial_names:
            output.write(f"    packet >> {name};\n")
        output.write(
            f"""}}

void {msg_name}Packet::Deserialize(const char* buf, size_t length) {{
    Packet packet;
    packet.append(buf, length);
    Deserialize(packet);
}}"""
        )
    os.rename(
        f"{msg_name}Packet.cpp", f"{output_dir}/cpp/communications/{msg_name}Packet.cpp"
    )


def write_packettype_header(output_dir, msg_names):
    """Write PacketType.hpp header file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_names -- list of packet message names
    """
    with open("PacketType.hpp", "w") as output:
        output.write(
            """#pragma once

#include <stdint.h>

namespace frc3512 {

"""
        )

        enum_type = "enum class PacketType : int8_t"
        types = ["k" + x for x in msg_names]
        singleline_types = ", ".join(types)

        len_first_line = len(enum_type) + len(singleline_types) + len(" {  };")
        if len_first_line <= 80:
            output.write(f"{enum_type} {{ {singleline_types} }};\n")
        else:
            multiline_types = ",".join(["\n    " + x for x in types])
            output.write(f"{enum_type} {{{multiline_types}\n}};\n")
        output.write(
            """
}  // namespace frc3512
"""
        )
    os.rename("PacketType.hpp", f"{output_dir}/include/communications/PacketType.hpp")


def write_publishnodebase_header(output_dir, msg_names):
    """Write PublishNodeBase.hpp header file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_names -- list of packet message names
    """
    with open("PublishNodeBase.hpp", "w") as output:
        output.write(
            """#pragma once

#include <stdint.h>

#include <mutex>

#include <wpi/SmallVector.h>

"""
        )
        for msg_name in msg_names:
            output.write(f'#include "communications/{msg_name}Packet.hpp"\n')
        output.write(
            """
namespace frc3512 {

class PublishNodeBase {
 public:
  /**
   * Deserialize the provided message and process it via the ProcessMessage()
   * function corresponding to the message type.
   *
   * Do NOT provide an implementation for this function. messages.py generates
   * one in PacketType.cpp.
   *
   * @param message The buffer containing the message to deserialize.
   */
  void DeserializeAndProcessMessage(wpi::SmallVectorImpl<char>& message);

"""
        )
        for msg_name in msg_names:
            output.write(
                f"  virtual void ProcessMessage(const {msg_name}Packet& message);\n"
            )
        output.write(
            """
  protected:
    std::mutex m_mutex;
};

}  // namespace frc3512
"""
        )
    os.rename(
        "PublishNodeBase.hpp",
        f"{output_dir}/include/communications/PublishNodeBase.hpp",
    )


def write_publishnodebase_source(output_dir, msg_names):
    """Write PublishNodeBase.cpp source file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_names -- list of packet message names
    """
    with open("PublishNodeBase.cpp", "w") as output:
        output.write(
            """
#include "communications/PublishNodeBase.hpp"

using namespace frc3512;

void PublishNodeBase::DeserializeAndProcessMessage(wpi::SmallVectorImpl<char>& message) {
    // Checks the first byte of the message for its ID to determine
    // which packet to deserialize to, then processes it
    auto packetType = static_cast<PacketType>(message[0]);
"""
        )
        for i, msg_name in enumerate(msg_names):
            if i == 0:
                output.write("    if ")
            else:
                output.write(" else if ")
            output.write(f"(packetType == PacketType::k{msg_name}) " "{\n")
            output.write(f"        {msg_name}Packet packet;" "\n")
            output.write(
                """        packet.Deserialize(message.data(), message.size());
        m_mutex.unlock();
        ProcessMessage(packet);
        m_mutex.lock();
    }"""
            )
        output.write(
            """
}
"""
        )
        for msg_name in msg_names:
            output.write("\n")
            output.write(
                f"void PublishNodeBase::ProcessMessage(const {msg_name}Packet& message) {{}}\n"
            )
    os.rename(
        "PublishNodeBase.cpp", f"{output_dir}/cpp/communications/PublishNodeBase.cpp"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Parses message descriptor files from the given directory and generates C++ source for serializing and deserializing them."
    )
    parser.add_argument("--input", help="directory containing message files")
    parser.add_argument("--output", help="directory to which to write C++ source")
    args = parser.parse_args()

    msg_files = [
        os.path.join(dp, f)
        for dp, dn, fn in os.walk(args.input)
        for f in fn
        if f.endswith(".msg")
    ]

    # Make destination folders for messages
    if not os.path.exists(f"{args.output}/cpp/communications"):
        os.makedirs(f"{args.output}/cpp/communications")
    if not os.path.exists(f"{args.output}/include/communications"):
        os.makedirs(f"{args.output}/include/communications")

    # Parse schema files
    var_regex = re.compile(r"(?P<type>\w+)\s+(?P<name>\w+)")
    for filename in msg_files:
        with open(filename, "r") as msgfile:
            member_var_types = ["std::string"]
            member_var_names = ["topic"]
            constructor_arg_types = ["wpi::StringRef"]
            serial_names = ["ID", "topic"]
            for line in msgfile:
                # Strip comments
                if line.find("#") != -1:
                    line = line[: line.find("#")]

                match = var_regex.search(line)
                if match:
                    type = match.group("type")
                    name = match.group("name")
                    if type == "string":
                        member_var_types.append("std::string")
                        constructor_arg_types.append("wpi::StringRef")
                    else:
                        member_var_types.append(type)
                        constructor_arg_types.append(type)
                    member_var_names.append(name)
                    serial_names.append(name)

            msg_name = os.path.splitext(os.path.basename(filename))[0]
            write_msg_header(
                args.output,
                msg_name,
                member_var_types,
                constructor_arg_types,
                member_var_names,
            )
            write_msg_source(
                args.output,
                msg_name,
                constructor_arg_types,
                member_var_names,
                serial_names,
            )
    msg_names = [os.path.splitext(os.path.basename(name))[0] for name in msg_files]
    msg_names = sorted(msg_names)
    write_packettype_header(args.output, msg_names)
    write_publishnodebase_header(args.output, msg_names)
    write_publishnodebase_source(args.output, msg_names)


if __name__ == "__main__":
    main()
