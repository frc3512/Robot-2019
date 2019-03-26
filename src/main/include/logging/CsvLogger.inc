// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {

template <class... doubles>
void CsvLogger::Log(double value, doubles... values) {
    m_logfile << Timestamp() << ',';

    LogImpl(value, values...);
}

template <class... doubles>
void CsvLogger::LogImpl(double value, doubles... values) {
    m_logfile << value << ',';
    LogImpl(values...);
}

}  // namespace frc3512
