program Viewer;

{$IFDEF FPC}
  {$MODE Delphi}
{$ENDIF}

uses
CThreads,
{$IFnDEF FPC}
{$ELSE}
  Interfaces,
{$ENDIF}
  Forms,
  Unit1 in 'Unit1.pas' {Form1};


{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TForm1, Form1);
  Application.Run;
end.
