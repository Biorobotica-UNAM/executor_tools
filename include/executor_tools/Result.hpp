#pragma once

#include <string>

struct Result
{
    bool        success;   // true = SUCCESS, false = FAILURE
    std::string state;     // último estado de la FSM ("SUCCESS", "FAILED", ...)
    std::string message;   // contexto para el supervisor (qué pasó y por qué)
};
