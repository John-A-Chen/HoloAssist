import { useEffect } from 'react';

import { useRobotStore } from '../store/useRobotStore';

function isTypingTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) {
    return false;
  }
  const tag = target.tagName;
  return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || target.isContentEditable;
}

export function useKeyboardShortcuts(): void {
  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (isTypingTarget(event.target)) {
        return;
      }

      if (event.code === 'Space') {
        event.preventDefault();
        useRobotStore.getState().setDeadmanHeld(true);
        return;
      }

      if (event.code === 'KeyE' && !event.repeat) {
        event.preventDefault();
        const state = useRobotStore.getState();
        if (state.connectionStatus === 'Connected') {
          state.openEstopModal();
        }
        return;
      }

      if (event.code === 'KeyR' && !event.repeat) {
        event.preventDefault();
        void useRobotStore.getState().resetFault();
      }
    };

    const onKeyUp = (event: KeyboardEvent) => {
      if (event.code === 'Space') {
        event.preventDefault();
        useRobotStore.getState().setDeadmanHeld(false);
      }
    };

    const onBlur = () => {
      useRobotStore.getState().setDeadmanHeld(false);
    };

    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    window.addEventListener('blur', onBlur);

    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      window.removeEventListener('blur', onBlur);
    };
  }, []);
}
