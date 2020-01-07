// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {

template <class... doubles>
void CsvLogger::Log(double value, doubles... values) {
    m_logfile << Timestamp() << ',';

    WriteValues(value, values...);
}

template <class... doubles>
void CsvLogger::Log(units::second_t t, double value, doubles... values) {
    m_logfile << t.to<double>() << ',';

    WriteValues(value, values...);
}

template <class... doubles>
void CsvLogger::WriteValues(double value, doubles... values) {
    m_logfile << value << ',';
    if constexpr (sizeof...(values) > 0) {
        WriteValues(values...);
    } else {
        m_logfile.flush();
    }
}

}  // namespace frc3512
