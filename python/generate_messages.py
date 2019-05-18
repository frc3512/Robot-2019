#!/usr/bin/env python3

"""Called by generate_src.py."""

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
        output.write("\n")
        output.write(f"    {msg_name}Packet() = default;\n")
        output.write("\n")
        output.write("    /**\n")
        output.write(f"     * Construct a {msg_name}Packet with the given fields.\n")
        output.write("     */\n")
        output.write(f"    {msg_name}Packet(")
        output.write(
            ", ".join(
                [
                    x[0] + " " + x[1]
                    for x in zip(constructor_arg_types, member_var_names)
                ]
            )
        )
        output.write(");\n")
        output.write("\n")
        output.write("    /**\n")
        output.write("     * Deserializes the given packet\n")
        output.write("     *\n")
        output.write("     * @param packet The packet to deserialize\n")
        output.write("     */\n")
        output.write(f"    {msg_name}Packet(Packet& packet);\n")
        output.write("\n")
        output.write("    /**\n")
        output.write("     * Serializes the given packet.\n")
        output.write("     *\n")
        output.write(
            "     * The contents of the packet should be passed to whatever communication\n"
        )
        output.write("     * layer that takes a raw buffer\n")
        output.write("     */\n")
        output.write("    Packet Serialize() const;\n")
        output.write("\n")
        output.write("    /**\n")
        output.write("     * Deserializes the given packet.\n")
        output.write("     *\n")
        output.write("     * @param packet The buffer containing the packet.\n")
        output.write("     * @param size   The length of the packet.\n")
        output.write("     */\n")
        output.write("    void Deserialize(const char* buf, size_t size);\n")
        output.write("\n")
        output.write("    /**\n")
        output.write("     * Deserializes the given packet.\n")
        output.write("     *\n")
        output.write("     * @param packet The packet to deserialize.\n")
        output.write("     */\n")
        output.write("    void Deserialize(Packet& packet);\n")
        output.write("};\n")
        output.write("\n")
        output.write("}\n")
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
        output.write(f'#include "communications/{msg_name}Packet.hpp"\n')
        output.write("\n")
        output.write("using namespace frc3512;\n")
        output.write("\n")
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
        output.write("}\n")
        output.write("\n")
        output.write(f"{msg_name}Packet::{msg_name}Packet(Packet& packet) {{\n")
        output.write("    Deserialize(packet);\n")
        output.write("}\n")
        output.write("\n")
        output.write(f"Packet {msg_name}Packet::Serialize() const {{\n")
        output.write("    Packet packet;\n")
        for name in serial_names:
            output.write(f"    packet << {name};\n")
        output.write("    return packet;\n")
        output.write("}\n")
        output.write("\n")
        output.write(f"void {msg_name}Packet::Deserialize(Packet& packet) {{\n")
        for name in serial_names:
            output.write(f"    packet >> {name};\n")
        output.write("}\n")
        output.write("\n")
        output.write(
            f"void {msg_name}Packet::Deserialize(const char* buf, size_t length) {{\n"
        )
        output.write("    Packet packet;\n")
        output.write("    packet.append(buf, length);\n")
        output.write("    Deserialize(packet);\n")
        output.write("}\n")
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
        output.write("#pragma once\n")
        output.write("\n")
        output.write("#include <stdint.h>\n")
        output.write("\n")
        output.write("namespace frc3512 {\n")
        output.write("\n")

        enum_type = "enum class PacketType : int8_t"
        types = ["k" + x for x in msg_names]
        singleline_types = ", ".join(types)

        len_first_line = len(enum_type) + len(singleline_types) + len(" {  };")
        if len_first_line <= 80:
            output.write(f"{enum_type} {{ {singleline_types} }};\n")
        else:
            multiline_types = ",".join(["\n    " + x for x in types])
            output.write(f"{enum_type} {{{multiline_types}\n}};\n")
        output.write("\n")
        output.write("}\n")
    os.rename("PacketType.hpp", f"{output_dir}/include/communications/PacketType.hpp")


def write_publishnodebase_header(output_dir, msg_names):
    """Write PublishNodeBase.hpp header file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_names -- list of packet message names
    """
    with open("PublishNodeBase.hpp", "w") as output:
        output.write("#pragma once\n")
        output.write("\n")
        output.write("#include <stdint.h>\n")
        output.write("\n")
        output.write("#include <mutex>\n")
        output.write("\n")
        output.write("#include <wpi/SmallVector.h>\n")
        output.write("\n")
        for msg_name in msg_names:
            output.write(f'#include "communications/{msg_name}Packet.hpp"\n')
        output.write("\n")
        output.write("namespace frc3512 {\n")
        output.write("\n")

        output.write("class PublishNodeBase {\n")
        output.write(" public:\n")
        output.write(
            """  /**
   * Deserialize the provided message and process it via the ProcessMessage()
   * function corresponding to the message type.
   *
   * Do NOT provide an implementation for this function. messages.py generates
   * one in PacketType.cpp.
   *
   * @param message The buffer containing the message to deserialize.
   */\n"""
        )
        output.write(
            "  void DeserializeAndProcessMessage(wpi::SmallVectorImpl<char>& message);\n"
        )
        output.write("\n")
        for msg_name in msg_names:
            output.write(
                f"  virtual void ProcessMessage(const {msg_name}Packet& message);\n"
            )
        output.write("\n")
        output.write("  protected:\n")
        output.write("    std::mutex m_mutex;\n")
        output.write("};\n")
        output.write("\n")
        output.write("}  // namespace frc3512\n")
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
        output.write('#include "communications/PublishNodeBase.hpp"\n')
        output.write("\n")
        output.write("using namespace frc3512;\n")
        output.write("\n")
        output.write(
            "void PublishNodeBase::DeserializeAndProcessMessage(wpi::SmallVectorImpl<char>& message) {\n"
        )
        output.write(
            "    // Checks the first byte of the message for its ID to determine\n"
        )
        output.write("    // which packet to deserialize to, then processes it\n")
        output.write("    auto packetType = static_cast<PacketType>(message[0]);\n")
        for i, msg_name in enumerate(msg_names):
            if i == 0:
                output.write("    if ")
            else:
                output.write(" else if ")
            output.write(f"(packetType == PacketType::k{msg_name}) " "{\n")
            output.write(f"        {msg_name}Packet packet;" "\n")
            output.write(
                f"        packet.Deserialize(message.data(), message.size());\n"
            )
            output.write("        m_mutex.unlock();\n")
            output.write("        ProcessMessage(packet);\n")
            output.write("        m_mutex.lock();\n")
            output.write("    }")
        output.write("\n}\n")
        output.write("\n")
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
    parser.add_argument(
        "--input", dest="input_dir", help="directory containing message files"
    )
    parser.add_argument(
        "--output", dest="output_dir", help="directory to which to write C++ source"
    )
    args = parser.parse_args()

    msg_files = [
        os.path.join(dp, f)
        for dp, dn, fn in os.walk(args.input_dir)
        for f in fn
        if f.endswith(".msg")
    ]

    # Make destination folders for messages
    if not os.path.exists(f"{args.output_dir}/cpp/communications"):
        os.makedirs(f"{args.output_dir}/cpp/communications")
    if not os.path.exists(f"{args.output_dir}/include/communications"):
        os.makedirs(f"{args.output_dir}/include/communications")

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
                args.output_dir,
                msg_name,
                member_var_types,
                constructor_arg_types,
                member_var_names,
            )
            write_msg_source(
                args.output_dir,
                msg_name,
                constructor_arg_types,
                member_var_names,
                serial_names,
            )
    msg_names = [os.path.splitext(os.path.basename(name))[0] for name in msg_files]
    msg_names = sorted(msg_names)
    write_packettype_header(args.output_dir, msg_names)
    write_publishnodebase_header(args.output_dir, msg_names)
    write_publishnodebase_source(args.output_dir, msg_names)


if __name__ == "__main__":
    main()
