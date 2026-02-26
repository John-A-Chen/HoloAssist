import type { ReactNode } from 'react';

import { Button } from './button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from './card';

interface ModalProps {
  open: boolean;
  title: string;
  description?: string;
  confirmLabel?: string;
  cancelLabel?: string;
  confirmVariant?: 'default' | 'destructive' | 'success';
  onCancel: () => void;
  onConfirm: () => void;
  children?: ReactNode;
}

export function ConfirmModal({
  open,
  title,
  description,
  confirmLabel = 'Confirm',
  cancelLabel = 'Cancel',
  confirmVariant = 'default',
  onCancel,
  onConfirm,
  children
}: ModalProps) {
  if (!open) {
    return null;
  }

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-slate-950/80 p-4 backdrop-blur-sm">
      <Card className="w-full max-w-md border-slate-700 bg-slate-900">
        <CardHeader className="pb-1">
          <CardTitle>{title}</CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          {description ? <CardDescription className="text-sm leading-6 text-slate-300">{description}</CardDescription> : null}
          {children}
          <div className="flex justify-end gap-2">
            <Button variant="outline" onClick={onCancel}>
              {cancelLabel}
            </Button>
            <Button variant={confirmVariant} onClick={onConfirm}>
              {confirmLabel}
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
