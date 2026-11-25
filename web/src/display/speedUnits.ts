import { createContext, useContext } from 'react';
export type SpeedUnit = 'm/s' | 'km/h';
export const SpeedUnitContext = createContext<SpeedUnit>('m/s');
export const useSpeedUnit = (): SpeedUnit => useContext(SpeedUnitContext);
export const speedScale = (unit: SpeedUnit): number => unit === 'km/h' ? 3.6 : 1;
export const displaySpeed = (si: number, unit: SpeedUnit): number => si * speedScale(unit);
export const speedSI = (value: number, unit: SpeedUnit): number => value / speedScale(unit);
